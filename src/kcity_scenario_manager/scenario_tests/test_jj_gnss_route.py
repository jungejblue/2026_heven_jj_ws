"""Offline route/georeference regression tests and an optional live export test."""

import hashlib
import math
import os
from pathlib import Path
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from kcity_scenario_manager.tools.route.jj_gnss_route import (
    CANONICAL_MAP, convert_rows, generate, jj_local_xy, render_jj_csv, validate_map,
)
from kcity_scenario_manager.route_csv import read_route_csv


ROUTES = Path(__file__).resolve().parents[1] / 'routes'
SOURCE = ROUTES / 'qualifier.csv'
GENERATED = ROUTES / 'qualifier_jj_gnss.csv'
# Provenance of the bundled GNSS reference, not an accepted live-map origin.
RECORDED_ORIGIN = (37.24273, 126.77363)
REPOSITORY_MAP_ORIGIN = (37.2427, 126.773665)


def reference(latitude=37.2427, longitude=126.773665):
    return (f'+proj=tmerc +lat_0={latitude} +lon_0={longitude} +k=1 '
            '+x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs')


class FakeMap:
    name = CANONICAL_MAP

    def __init__(self, geo_reference=None, cdata=True):
        text = reference() if geo_reference is None else geo_reference
        content = f'<![CDATA[{text}]]>' if cdata else text
        self.xml = f'<OpenDRIVE><header><geoReference>{content}</geoReference></header></OpenDRIVE>'
        self.locations = []

    def to_opendrive(self):
        return self.xml

    def transform_to_geolocation(self, location):
        self.locations.append((location.x, location.y, location.z))
        return SimpleNamespace(latitude=37.239123456789, longitude=126.774987654321)


def fake_conversion_modules():
    carla = ModuleType('carla')
    carla.Location = lambda **values: SimpleNamespace(**values)
    agent = ModuleType('kcity_scenario_manager.csv_route_agent')
    agent.reconstruct_csv_waypoint = lambda *_args: (None, None, 0.005)
    return {'carla': carla, agent.__name__: agent}


class MapGeoreferenceTests(unittest.TestCase):
    def test_recorded_and_repository_origins_are_both_valid(self):
        for origin in (RECORDED_ORIGIN, REPOSITORY_MAP_ORIGIN):
            for cdata in (True, False):
                with self.subTest(origin=origin, cdata=cdata):
                    text = reference(*origin)
                    self.assertEqual(validate_map(FakeMap(text, cdata)), text)

    def test_equivalent_numeric_formatting_is_valid(self):
        text = reference().replace('+k=1 ', '+k=1.000 ').replace('+x_0=0 ', '+x_0=0.0 ')
        self.assertEqual(validate_map(FakeMap(text)), text)

    def test_wrong_map_rejected(self):
        carla_map = FakeMap()
        carla_map.name = 'other/map'
        with self.assertRaisesRegex(RuntimeError, 'canonical map'):
            validate_map(carla_map)

    def test_missing_empty_and_malformed_georeference_rejected(self):
        for xml in ('<OpenDRIVE/>', FakeMap('').xml, '<OpenDRIVE>'):
            with self.subTest(xml=xml):
                carla_map = FakeMap()
                carla_map.xml = xml
                with self.assertRaisesRegex(RuntimeError, 'geoReference'):
                    validate_map(carla_map)

    def test_invalid_numeric_parameters_rejected(self):
        invalid = (
            reference(latitude='NaN'), reference(longitude='inf'),
            reference(latitude=90), reference(latitude=-91),
            reference(longitude=181), reference(longitude='bad'),
            reference().replace('+lat_0=37.2427 ', ''),
            reference().replace('+k=1 ', '+k=0.9996 '),
            reference().replace('+x_0=0 ', '+x_0=100 '),
        )
        for text in invalid:
            with self.subTest(reference=text):
                with self.assertRaisesRegex(RuntimeError, 'geoReference'):
                    validate_map(FakeMap(text))

    def test_unsupported_metadata_and_duplicate_origin_rejected(self):
        for text in (
            reference().replace('+proj=tmerc', '+proj=utm'),
            reference().replace('+datum=WGS84', '+datum=NAD83'),
            reference().replace('+units=m', '+units=ft'),
            reference() + ' +lat_0=0',
        ):
            with self.subTest(reference=text):
                with self.assertRaisesRegex(RuntimeError, 'geoReference'):
                    validate_map(FakeMap(text))


class RouteExportTests(unittest.TestCase):
    def test_bundled_qualifier_contract_and_recorded_geometry(self):
        rows = read_route_csv(SOURCE)
        lines = GENERATED.read_text(encoding='ascii').splitlines()
        self.assertEqual(len(rows), 499)
        self.assertEqual(len(lines), len(rows))
        self.assertEqual(lines[0].count(','), 1)
        self.assertFalse(any(character.isalpha() for character in lines[0]))
        for row, line in zip(rows, lines):
            latitude, longitude = (float(value) for value in line.split(','))
            self.assertTrue(37 < latitude < 38 and 126 < longitude < 127)
            x, y = jj_local_xy(latitude, longitude, *RECORDED_ORIGIN)
            self.assertLess(math.hypot(x-float(row['x']), y+float(row['y'])), 0.02)

    def test_local_projection_requires_explicit_origin(self):
        heven_origin = (37.2388873, 126.7729325)
        self.assertEqual(jj_local_xy(*heven_origin, *heven_origin), (0.0, 0.0))
        self.assertGreater(math.hypot(*jj_local_xy(*heven_origin, *RECORDED_ORIGIN)), 400)

    def test_headerless_deterministic_output(self):
        coordinates = [(37.1, 126.2), (37.100001, 126.200001)]
        self.assertEqual(render_jj_csv(coordinates), render_jj_csv(list(coordinates)))
        self.assertEqual(render_jj_csv(coordinates),
                         '37.100000000000,126.200000000000\n37.100001000000,126.200001000000\n')

    def test_bad_reference_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            malformed = Path(directory) / 'route.csv'
            malformed.write_text('x,y\n1,2\n', encoding='utf-8')
            with self.assertRaisesRegex(RuntimeError, 'missing columns'):
                read_route_csv(malformed)
        with self.assertRaisesRegex(RuntimeError, 'at least two'):
            render_jj_csv([(37.0, 126.0)])

    def test_conversion_preserves_server_geolocation_and_native_coordinates(self):
        rows = read_route_csv(SOURCE)[:2]
        for origin in (RECORDED_ORIGIN, REPOSITORY_MAP_ORIGIN):
            with self.subTest(origin=origin), patch.dict('sys.modules', fake_conversion_modules()):
                carla_map = FakeMap(reference(*origin))
                result, error = convert_rows(carla_map, rows, 2)
                self.assertEqual(result, [(37.239123456789, 126.774987654321)] * 2)
                self.assertEqual(carla_map.locations, [(row['x'], row['y'], row['z']) for row in rows])
                self.assertEqual(error, 0.005)

    def test_route_geometry_check_and_point_count_are_preserved(self):
        rows = read_route_csv(SOURCE)[:2]
        modules = fake_conversion_modules()

        def mismatch(*_args):
            raise RuntimeError('waypoint mismatch')

        modules['kcity_scenario_manager.csv_route_agent'].reconstruct_csv_waypoint = mismatch
        with patch.dict('sys.modules', modules):
            with self.assertRaisesRegex(RuntimeError, 'waypoint mismatch'):
                convert_rows(FakeMap(), rows, 2)
            with self.assertRaisesRegex(RuntimeError, 'expected 499'):
                convert_rows(FakeMap(), rows, 499)

    def test_invalid_geolocation_does_not_overwrite_output(self):
        carla_map = FakeMap()
        carla_map.transform_to_geolocation = lambda _location: SimpleNamespace(
            latitude=math.nan, longitude=126.0)
        with tempfile.TemporaryDirectory() as directory, patch.dict('sys.modules', fake_conversion_modules()):
            output = Path(directory) / 'output.csv'
            output.write_text('existing output\n')
            with self.assertRaisesRegex(RuntimeError, 'invalid CARLA geolocation'):
                generate(carla_map, SOURCE, output)
            self.assertEqual(output.read_text(), 'existing output\n')

    def test_generated_file_matches_recorded_source_digest(self):
        self.assertEqual(hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
                         '91be54184807d600d20056f2df7f9387509a257728687dd249da5a64278bd9fc')

    @unittest.skipUnless(os.environ.get('KCITY_CARLA_LIVE_TEST') == '1',
                         'set KCITY_CARLA_LIVE_TEST=1 with canonical CARLA server')
    def test_live_carla_conversion_is_deterministic_for_current_map(self):
        import carla
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10)
        carla_map = client.get_world().get_map()
        with tempfile.TemporaryDirectory() as directory:
            first, second = Path(directory) / 'first.csv', Path(directory) / 'second.csv'
            result_a = generate(carla_map, SOURCE, first, 499)
            result_b = generate(carla_map, SOURCE, second, 499)
            self.assertEqual(first.read_bytes(), second.read_bytes())
            self.assertEqual(result_a, result_b)
            self.assertLess(result_a['max_xodr_error_m'], 0.01)
            for row, line in zip(read_route_csv(SOURCE), first.read_text().splitlines()):
                geo = carla_map.transform_to_geolocation(carla.Location(x=row['x'], y=row['y'], z=row['z']))
                lat, lon = (float(value) for value in line.split(','))
                self.assertAlmostEqual(lat, geo.latitude, places=11)
                self.assertAlmostEqual(lon, geo.longitude, places=11)


if __name__ == '__main__':
    unittest.main()
