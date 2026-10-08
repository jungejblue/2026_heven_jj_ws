"""CSV helpers shared by route generation, benchmark loading, and smoke driving."""

from __future__ import annotations

import csv
import hashlib
import math
from pathlib import Path


CSV_COLUMNS = (
    'index',
    'route_s_m',
    'x',
    'y',
    'z',
    'yaw',
    'road_id',
    'section_id',
    'lane_id',
    'opendrive_s',
    'is_junction',
    'road_option',
)

REQUIRED_CSV_COLUMNS=tuple(
    column for column in CSV_COLUMNS if column!='road_option'
)

FLOAT_COLUMNS = ('route_s_m','x','y','z','yaw','opendrive_s')
INT_COLUMNS = ('index','road_id','section_id','lane_id')


def resolve_route_csv_path(config_file,route_csv):
    value=Path(str(route_csv)).expanduser()
    if value.is_absolute():
        return value
    config_path=Path(config_file).expanduser().resolve()
    candidates=(config_path.parent/value,config_path.parent.parent/value)
    for candidate in candidates:
        if candidate.exists():
            return candidate
    if value.parts and value.parts[0]=='routes':
        return candidates[1]
    return candidates[0]


def parse_bool(value):
    text=str(value).strip().lower()
    if text in {'1','true','yes'}:
        return True
    if text in {'0','false','no'}:
        return False
    raise ValueError(f'invalid boolean value: {value!r}')


def validate_route_rows(rows):
    if len(rows)<2:
        raise RuntimeError('route CSV must contain at least two points')
    previous_s=None
    for expected_index,row in enumerate(rows):
        if int(row['index'])!=expected_index:
            raise RuntimeError(
                f'route CSV index mismatch at row {expected_index}: {row["index"]}'
            )
        values=[float(row[key]) for key in FLOAT_COLUMNS]
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError(f'route CSV contains NaN/Inf at index {expected_index}')
        route_s=float(row['route_s_m'])
        if previous_s is not None and route_s<previous_s:
            raise RuntimeError(
                f'route_s_m decreases at index {expected_index}'
            )
        previous_s=route_s
    return rows


def read_route_csv(path):
    csv_path=Path(path).expanduser()
    with csv_path.open(newline='',encoding='utf-8') as stream:
        reader=csv.DictReader(stream)
        missing=[
            column for column in REQUIRED_CSV_COLUMNS
            if column not in (reader.fieldnames or [])
        ]
        if missing:
            raise RuntimeError(
                f'route CSV is missing columns: {", ".join(missing)}'
            )
        rows=[]
        for raw in reader:
            row={
                key:(raw.get(key,'LANEFOLLOW') if key=='road_option' else raw[key])
                for key in CSV_COLUMNS
            }
            for key in INT_COLUMNS:
                row[key]=int(row[key])
            for key in FLOAT_COLUMNS:
                row[key]=float(row[key])
            row['is_junction']=parse_bool(row['is_junction'])
            rows.append(row)
    return validate_route_rows(rows)


def write_route_csv(path,rows):
    csv_path=Path(path).expanduser()
    validate_route_rows(rows)
    csv_path.parent.mkdir(parents=True,exist_ok=True)
    with csv_path.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for row in rows:
            writer.writerow({
                key:(row.get(key,'LANEFOLLOW') if key=='road_option' else row[key])
                for key in CSV_COLUMNS
            })
    return csv_path


def route_sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def route_locations(rows):
    return [(float(row['x']),float(row['y']),float(row['z'])) for row in rows]


def downsample_locations(locations,spacing_m):
    points=list(locations)
    if len(points)<2:
        raise RuntimeError('route requires at least two locations')
    spacing=float(spacing_m)
    if spacing<=0:
        raise ValueError('spacing_m must be positive')
    selected=[points[0]]
    accumulated=0.0
    previous=points[0]
    for point in points[1:-1]:
        accumulated+=math.dist(previous,point)
        previous=point
        if accumulated>=spacing:
            selected.append(point)
            accumulated=0.0
    if selected[-1]!=points[-1]:
        selected.append(points[-1])
    return selected
