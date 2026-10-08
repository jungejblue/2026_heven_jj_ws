"""Read-only CARLA map query to generate a route-aligned ego spawn JSON."""

import argparse
import json
from pathlib import Path

from .route_spawn_math import build_spawn_config, load_latlon_csv


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True, help="HEVEN latitude,longitude route CSV")
    parser.add_argument("--output", required=True, help="Output carla_spawn_objects JSON")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--z", type=float, default=4.0, help="Initial native/ROS spawn height in metres")
    parser.add_argument("--lookahead-m", type=float, default=5.0, help="Initial route tangent distance")
    options = parser.parse_args(args)
    points = load_latlon_csv(options.csv)
    import carla
    client = carla.Client(options.host, options.port)
    client.set_timeout(5.0)
    carla_map = client.get_world().get_map()

    def geolocate(x, y, z):
        geo = carla_map.transform_to_geolocation(carla.Location(x=x, y=y, z=z))
        return geo.latitude, geo.longitude, geo.altitude

    config = build_spawn_config(points, geolocate, options.z, options.lookahead_m)
    output = Path(options.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    spawn = config["objects"][0]["spawn_point"]
    print(f"Wrote {output} from running map {carla_map.name}: "
          f"ROS x={spawn['x']:.3f}, y={spawn['y']:.3f}, yaw={spawn['yaw']:.3f} deg")


if __name__ == "__main__":
    main()
