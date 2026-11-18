"""Read-only RM65-BI/API 4.2.8 connection check; never sends motion or stop commands."""

import argparse
import math

from Realman_Control.legacy_sdk import SDK_VERSION
from Realman_Control.realman_control import RM65BI


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", default="192.168.1.18")
    parser.add_argument("--robot-port", type=int, default=8080)
    args = parser.parse_args(argv)
    robot = RM65BI(args.robot_ip, port=args.robot_port)
    try:
        print(f"Control SDK: RealMan API {SDK_VERSION}")
        print(f"Controller: {robot.get_software_info()}")
        print(f"Joints (deg): {[round(math.degrees(q), 3) for q in robot.get_joint_values()]}")
        limits = robot.get_joint_limits()
        print(f"Joint limits (deg): {[(round(math.degrees(lo), 3), round(math.degrees(hi), 3)) for lo, hi in limits]}")
        print(f"TCP (m + rotation vector rad): {robot.get_tool_pose()}")
        print(f"Tool frame (m + Euler rad): {robot.get_active_tool_frame_pose_euler()}")
        print(f"Offline safety Algo: {robot._safety_algo().rm_algo_version()}")
        print("Connection and read-only checks passed.")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
