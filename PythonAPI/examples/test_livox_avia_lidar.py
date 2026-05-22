#!/usr/bin/env python3

import argparse
import time

import numpy as np
import carla


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=2000, type=int)
    parser.add_argument("--duration", default=5.0, type=float)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    bp = bp_lib.find("sensor.lidar.livox_avia")

    vehicle_bp = bp_lib.filter("vehicle.*")[0]
    spawn_point = world.get_map().get_spawn_points()[0]
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)

    lidar_tf = carla.Transform(
        carla.Location(x=0.0, y=0.0, z=2.0),
        carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0),
    )

    lidar = world.spawn_actor(bp, lidar_tf, attach_to=vehicle)

    stat = {
        "frames": 0,
        "points": 0,
    }

    def callback(data):
        arr = np.frombuffer(data.raw_data, dtype=np.float32)
        arr = arr.reshape((-1, 4))

        stat["frames"] += 1
        stat["points"] += arr.shape[0]

        if stat["frames"] <= 10:
            x, y, z = arr[:, 0], arr[:, 1], arr[:, 2]
            yaw = np.degrees(np.arctan2(y, x))
            pitch = np.degrees(np.arctan2(z, np.sqrt(x * x + y * y)))
            print(
                f"frame={data.frame}, "
                f"points={arr.shape[0]}, "
                f"channels={data.channels}, "
                f"yaw_fov≈{yaw.max() - yaw.min():.2f}, "
                f"pitch_fov≈{pitch.max() - pitch.min():.2f}"
            )

    lidar.listen(callback)

    try:
        start = time.time()
        while time.time() - start < args.duration:
            world.wait_for_tick()
    finally:
        lidar.stop()
        lidar.destroy()
        vehicle.destroy()

    print("frames:", stat["frames"])
    print("points:", stat["points"])
    print("approx points/s:", stat["points"] / args.duration)


if __name__ == "__main__":
    main()