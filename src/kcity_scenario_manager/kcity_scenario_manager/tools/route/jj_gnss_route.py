"""Export an exact K-City reference route for the unchanged JJ GNSS planner."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ...route_csv import read_route_csv, route_sha256


CANONICAL_MAP = 'heven_kcity/Maps/kcity/kcity'
EARTH_RADIUS_M = 6378135.0  # jj_planner and jj_localization source


def jj_local_xy(latitude, longitude, origin_lat, origin_lon):
    """Project absolute GNSS using the caller's explicit JJ map origin."""
    return (
        (longitude-origin_lon)*math.cos(math.radians(origin_lat))
        *math.pi*EARTH_RADIUS_M/180.0,
        (latitude-origin_lat)*math.pi*EARTH_RADIUS_M/180.0,
    )


def validate_map(carla_map):
    """Check supported georeferencing without pinning a map revision's origin.

    CARLA supplies the actual absolute geolocation. Route road/lane/s metadata
    is checked separately by reconstruct_csv_waypoint before conversion.
    """
    if carla_map.name != CANONICAL_MAP:
        raise RuntimeError(f'expected canonical map {CANONICAL_MAP}, got {carla_map.name}')
    try:
        root = ET.fromstring(carla_map.to_opendrive())
    except ET.ParseError as exc:
        raise RuntimeError('OpenDRIVE XML is invalid; cannot read geoReference') from exc
    reference_element = root.find('header/geoReference')
    if reference_element is None or not (reference_element.text or '').strip():
        raise RuntimeError('OpenDRIVE geoReference is missing')
    reference = reference_element.text.strip()
    tokens = reference.split()
    params = {}
    for token in tokens:
        if token.startswith('+') and '=' in token:
            key, value = token[1:].split('=', 1)
            if key in params:
                raise RuntimeError(f'duplicate geoReference parameter: {key}')
            params[key] = value
    expected = {'proj': 'tmerc', 'datum': 'WGS84', 'units': 'm', 'vunits': 'm'}
    if (any(params.get(key) != value for key, value in expected.items())
            or '+no_defs' not in tokens):
        raise RuntimeError(f'unsupported canonical map geoReference: {reference}')
    try:
        numeric = {key: float(params[key]) for key in ('lat_0', 'lon_0', 'k', 'x_0', 'y_0')}
    except (KeyError, ValueError) as exc:
        raise RuntimeError(f'invalid numeric geoReference parameters: {reference}') from exc
    if (not all(math.isfinite(value) for value in numeric.values())
            or not -90 < numeric['lat_0'] < 90
            or not -180 <= numeric['lon_0'] <= 180
            or numeric['k'] != 1.0 or numeric['x_0'] != 0.0 or numeric['y_0'] != 0.0):
        raise RuntimeError(f'invalid or unsupported geoReference parameters: {reference}')
    return reference


def convert_rows(carla_map, rows, expected_points):
    import carla
    from ...csv_route_agent import reconstruct_csv_waypoint

    if len(rows) != expected_points:
        raise RuntimeError(f'expected {expected_points} route points, got {len(rows)}')
    validate_map(carla_map)
    result = []
    xodr_errors = []
    for row in rows:
        _, _, error = reconstruct_csv_waypoint(carla_map, row, 0.01)
        xodr_errors.append(error)
        geo = carla_map.transform_to_geolocation(carla.Location(
            x=float(row['x']), y=float(row['y']), z=float(row['z'])))
        latitude, longitude = float(geo.latitude), float(geo.longitude)
        if not (math.isfinite(latitude) and math.isfinite(longitude)
                and -90 <= latitude <= 90 and -180 <= longitude <= 180):
            raise RuntimeError(f'index {row["index"]}: invalid CARLA geolocation')
        result.append((latitude, longitude))
    return result, max(xodr_errors)


def render_jj_csv(geolocations):
    if len(geolocations) < 2:
        raise RuntimeError('JJ route needs at least two points')
    return ''.join(f'{lat:.12f},{lon:.12f}\n' for lat, lon in geolocations)


def generate(carla_map, source, output, expected_points=499):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output:
        raise ValueError('source and output paths must differ')
    rows = read_route_csv(source)
    geolocations, max_xodr_error = convert_rows(carla_map, rows, expected_points)
    content = render_jj_csv(geolocations)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(content, encoding='ascii', newline='')
    return {'source_sha256': route_sha256(source), 'output_sha256': route_sha256(output),
            'count': len(geolocations), 'max_xodr_error_m': max_xodr_error,
            'first': geolocations[0], 'last': geolocations[-1]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-points', type=int, default=499)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=2000)
    args = parser.parse_args()
    if args.expected_points < 2:
        parser.error('--expected-points must be at least 2')
    import carla
    client = carla.Client(args.host, args.port)
    client.set_timeout(10)
    carla_map = client.get_world().get_map()
    result = generate(carla_map, args.source, args.output, args.expected_points)
    print(f'map={carla_map.name} source={args.source.resolve()} output={args.output.resolve()}')
    print(result)


if __name__ == '__main__':
    main()
