"""
Read-only utility: dump Spot's installed license so we can confirm which
features (e.g. Joint Level Control) are enabled.

A license read is a passive query: it needs no lease, no E-Stop, and never powers
motors. This utility must never move the robot.

Connection + auth reuse the project convention (env-based config, same as
main.py): SPOT_HOSTNAME / SPOT_USERNAME / SPOT_PASSWORD, loaded from server/.env.
Hostname may also be passed as a CLI argument, which overrides the env value.

Usage (from the server/ directory, with the venv active):
    python dump_license.py                         # host from SPOT_HOSTNAME
    python dump_license.py 192.168.80.3            # host from CLI
    python dump_license.py 192.168.80.3 --feature some-feature-code
    python dump_license.py --feature codeA --feature codeB
"""

import argparse
import os
import sys

from dotenv import load_dotenv

import bosdyn.client
from bosdyn.api import license_pb2
from bosdyn.client.license import LicenseClient

load_dotenv()


def _print_license_info(info: license_pb2.LicenseInfo) -> None:
    """Print the license both as its full protobuf text form and field-by-field,
    so the exact feature-code strings the robot reports are easy to read."""
    print("=== Installed license (full) ===")
    print(str(info).rstrip() or "(empty)")

    print("\n=== Installed license (fields) ===")
    print(f"status:       {info.Status.Name(info.status)}")
    print(f"id:           {info.id}")
    print(f"robot_serial: {info.robot_serial}")
    print(f"not_valid_before: {info.not_valid_before.ToDatetime() if info.not_valid_before.seconds else '-'}")
    print(f"not_valid_after:  {info.not_valid_after.ToDatetime() if info.not_valid_after.seconds else '-'}")

    print(f"licensed_features ({len(info.licensed_features)}):")
    if info.licensed_features:
        for code in info.licensed_features:
            print(f"  - {code}")
    else:
        print("  (none reported)")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Dump Spot's installed license (read-only; never moves the robot)."
    )
    parser.add_argument(
        "hostname",
        nargs="?",
        default=os.environ.get("SPOT_HOSTNAME"),
        help="Robot IP or hostname (defaults to SPOT_HOSTNAME from env/.env).",
    )
    parser.add_argument(
        "--feature",
        action="append",
        default=[],
        metavar="CODE",
        help="Feature code to check via get_feature_enabled (repeatable).",
    )
    args = parser.parse_args()

    if not args.hostname:
        print("No hostname given and SPOT_HOSTNAME is not set.", file=sys.stderr)
        return 2

    username = os.environ.get("SPOT_USERNAME", "user")
    password = os.environ.get("SPOT_PASSWORD")
    if not password:
        print("SPOT_PASSWORD is not set (check server/.env).", file=sys.stderr)
        return 2

    sdk = bosdyn.client.create_standard_sdk("spot-license-dump")
    robot = sdk.create_robot(args.hostname)

    try:
        robot.authenticate(username, password)
    except Exception as exc:
        print(
            f"Authentication/connection failed for {args.hostname}: {exc}\n"
            "Check the host is reachable (on Spot's wifi) and the credentials in "
            "server/.env are correct.",
            file=sys.stderr,
        )
        return 2

    try:
        license_client = robot.ensure_client(LicenseClient.default_service_name)
        info = license_client.get_license_info()
    except Exception as exc:
        print(f"License service unavailable: {exc}", file=sys.stderr)
        return 2

    _print_license_info(info)

    exit_code = 0
    if args.feature:
        try:
            # The client asserts feature_list is not a bare str, so pass a list.
            enabled = license_client.get_feature_enabled(list(args.feature))
        except Exception as exc:
            print(f"Feature check failed: {exc}", file=sys.stderr)
            return 2
        print("\n=== Requested feature checks ===")
        for code in args.feature:
            state = bool(enabled.get(code, False))
            print(f"{code}: {state}")
            if not state:
                exit_code = 1

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
