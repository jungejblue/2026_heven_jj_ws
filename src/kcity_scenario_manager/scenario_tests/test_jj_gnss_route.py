"""Qualifier reference to unchanged JJ planner route contract."""

import hashlib
import math
import os
from pathlib import Path

import pytest

from kcity_scenario_manager.tools.route.jj_gnss_route import (
    CANONICAL_MAP, ORIGIN_LATITUDE_DEG, ORIGIN_LONGITUDE_DEG,
    generate, jj_local_xy, render_jj_csv, validate_map,
)
from kcity_scenario_manager.route_csv import read_route_csv


ROUTES = Path(__file__).resolve().parents[1] / 'routes'
SOURCE = ROUTES / 'qualifier.csv'
GENERATED = ROUTES / 'qualifier_jj_gnss.csv'


def test_generated_qualifier_contract_and_geometry():
    rows = read_route_csv(SOURCE)
    lines = GENERATED.read_text(encoding='ascii').splitlines()
    assert len(rows) == len(lines) == 499
    assert lines[0].count(',') == 1
    assert not any(character.isalpha() for character in lines[0])
    positions = []
    for row, line in zip(rows, lines):
        latitude, longitude = (float(value) for value in line.split(','))
        assert 37 < latitude < 38 and 126 < longitude < 127  # latitude first
        x, y = jj_local_xy(latitude, longitude)
        positions.append((x, y))
        assert math.hypot(x-float(row['x']), y+float(row['y'])) < 0.02
        # Inverse of the exact spherical projection used by unchanged JJ nodes.
        restored_lat = ORIGIN_LATITUDE_DEG + y * 180 / (math.pi * 6378135.0)
        restored_lon = ORIGIN_LONGITUDE_DEG + x * 180 / (
            math.pi * 6378135.0 * math.cos(math.radians(ORIGIN_LATITUDE_DEG)))
        assert abs(restored_lat-latitude) < 1e-12
        assert abs(restored_lon-longitude) < 1e-12
    assert math.dist(positions[0], (float(rows[0]['x']), -float(rows[0]['y']))) < 0.02
    assert math.dist(positions[-1], (float(rows[-1]['x']), -float(rows[-1]['y']))) < 0.02


def test_output_is_headerless_and_deterministic():
    coordinates = [(37.1, 126.2), (37.100001, 126.200001)]
    assert render_jj_csv(coordinates) == render_jj_csv(list(coordinates))
    assert render_jj_csv(coordinates) == '37.100000000000,126.200000000000\n37.100001000000,126.200001000000\n'


def test_bad_reference_rejected(tmp_path):
    malformed = tmp_path / 'route.csv'
    malformed.write_text('x,y\n1,2\n', encoding='utf-8')
    with pytest.raises(RuntimeError, match='missing columns'):
        read_route_csv(malformed)
    with pytest.raises(RuntimeError, match='at least two'):
        render_jj_csv([(37.0, 126.0)])


def test_wrong_map_or_origin_rejected():
    class Map:
        name = CANONICAL_MAP

        def to_opendrive(self):
            return ('<geoReference><![CDATA[+proj=tmerc +lat_0=37.24273 '
                    '+lon_0=126.77363 +k=1 +x_0=0 +y_0=0 +datum=WGS84 '
                    '+units=m +vunits=m +no_defs]]></geoReference>')

    carla_map = Map()
    validate_map(carla_map)
    carla_map.name = 'other/map'
    with pytest.raises(RuntimeError, match='canonical map'):
        validate_map(carla_map)
    carla_map.name = CANONICAL_MAP
    carla_map.to_opendrive = lambda: '<geoReference><![CDATA[+proj=tmerc +lat_0=0 +lon_0=0 +datum=WGS84 +units=m]]></geoReference>'
    with pytest.raises(RuntimeError, match='geoReference'):
        validate_map(carla_map)


def test_live_carla_conversion_is_deterministic(tmp_path):
    if os.environ.get('KCITY_CARLA_LIVE_TEST') != '1':
        pytest.skip('set KCITY_CARLA_LIVE_TEST=1 with canonical CARLA server')
    import carla
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10)
    carla_map = client.get_world().get_map()
    first, second = tmp_path / 'first.csv', tmp_path / 'second.csv'
    result_a = generate(carla_map, SOURCE, first, 499)
    result_b = generate(carla_map, SOURCE, second, 499)
    assert first.read_bytes() == second.read_bytes() == GENERATED.read_bytes()
    assert result_a == result_b
    assert result_a['max_xodr_error_m'] < 0.01


def test_generated_file_matches_recorded_source_digest():
    assert hashlib.sha256(SOURCE.read_bytes()).hexdigest() == '91be54184807d600d20056f2df7f9387509a257728687dd249da5a64278bd9fc'
