"""Export an exact K-City reference route for the unchanged JJ GNSS planner."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import re

from ...route_csv import read_route_csv, route_sha256


CANONICAL_MAP = 'heven_kcity/Maps/kcity/kcity'
ORIGIN_LATITUDE_DEG = 37.24273
ORIGIN_LONGITUDE_DEG = 126.77363
EARTH_RADIUS_M = 6378135.0  # jj_planner and jj_localization source


def jj_local_xy(latitude, longitude, origin_lat=ORIGIN_LATITUDE_DEG,
                origin_lon=ORIGIN_LONGITUDE_DEG):
    return (
        (longitude-origin_lon)*math.cos(math.radians(origin_lat))
        *math.pi*EARTH_RADIUS_M/180.0,
        (latitude-origin_lat)*math.pi*EARTH_RADIUS_M/180.0,
    )


def validate_map(carla_map):
    if carla_map.name != CANONICAL_MAP:
        raise RuntimeError(f'expected canonical map {CANONICAL_MAP}, got {carla_map.name}')
    match = re.search(r'<geoReference>\s*<!\[CDATA\[(.*?)\]\]>\s*</geoReference>',
                      carla_map.to_opendrive(), re.DOTALL)
    if not match:
        raise RuntimeError('OpenDRIVE geoReference is missing')
    tokens = match.group(1).split()
    params = dict(token[1:].split('=', 1) for token in tokens if token.startswith('+') and '=' in token)
    expected = {'proj': 'tmerc', 'lat_0': str(ORIGIN_LATITUDE_DEG),
                'lon_0': str(ORIGIN_LONGITUDE_DEG), 'k': '1',
                'x_0': '0', 'y_0': '0', 'datum': 'WGS84',
                'units': 'm', 'vunits': 'm'}
    if (any(params.get(key) != value for key, value in expected.items())
            or '+no_defs' not in tokens):
        raise RuntimeError(f'unexpected canonical map geoReference: {match.group(1)}')
    return match.group(1)


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
