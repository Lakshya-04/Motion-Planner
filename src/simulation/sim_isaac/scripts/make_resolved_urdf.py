#!/usr/bin/env python3
"""Profile to a URDF the Isaac importer can read: xacro, package:// meshes copied next to it (needs ROS + xacro).

    make_resolved_urdf.py <profile.yaml> <out_dir>      # writes out_dir/robot.urdf and out_dir/<pkg>/...

Same two steps docker/tools/robot_assets.py does for the other generators, minus the arm-only variant.
"""
import importlib.util
import pathlib
import sys
import tempfile

import xacro
import yaml

REPO = next(p for p in pathlib.Path(__file__).resolve().parents if (p / 'docker' / 'tools').is_dir())
DESC = REPO / 'src' / 'description' / 'mobile_manipulator_description'


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    profile = yaml.safe_load(open(sys.argv[1]))
    args = load(DESC / 'scripts' / 'profile_xacro_args.py', 'profile_xacro_args').xacro_args(profile)
    out = pathlib.Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        raw = pathlib.Path(tmp) / 'robot.urdf'
        raw.write_text(xacro.process_file(str(DESC / 'urdf' / 'mobile_manipulator.urdf.xacro'), mappings=args).toxml())
        n = load(REPO / 'docker' / 'tools' / 'resolve_package_uris.py', 'resolve_package_uris').resolve(str(raw), str(out))
    print(f'wrote {out / "robot.urdf"} ({n} meshes copied)')


if __name__ == '__main__':
    main()
