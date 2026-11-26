#!/usr/bin/env python3
"""Read the `sensors:` block of a profile YAML and print one neutral dict per sensor as JSON.

    profile_sensors.py ur10e_husky_telescopic            # name in config/profiles, or a path
    profile_sensors.py profile.yaml --factory-only

The code lives in mobile_manipulator_description/sensors.py; this is the command line wrapper.
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))

from mobile_manipulator_description.sensors import (  # noqa: E402,F401
    SensorError, load_sensors, main, sensors_from_profile)

if __name__ == '__main__':
    sys.exit(main())
