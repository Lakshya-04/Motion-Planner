#!/usr/bin/env bash
# Turns a robot profile into the arm chain YAML and an RM4D map that standoff_sweep reads.
#
#   profile_to_map.sh <arm> <out_dir> [samples]
#
# <arm> is the xacro `arm:=` value of the profile (ur10e, xarm7, ...). Needs a sourced ROS 2 workspace
# with mobile_manipulator_description and reachability built (xacro, urdf_to_chain.py, rm4d_tool).
# Writes <out_dir>/robot.urdf, <out_dir>/<arm>_chain.yaml and <out_dir>/<arm>.rm4d. The map is the ARM's map
# (base frame arm_base_link, tool tcp_link); the lift and the base enter standoff_sweep through --profile.
set -euo pipefail
arm=${1:?usage: profile_to_map.sh <arm> <out_dir> [samples]}
out=${2:?usage: profile_to_map.sh <arm> <out_dir> [samples]}
samples=${3:-3000000}
mkdir -p "$out"
desc=$(ros2 pkg prefix mobile_manipulator_description)/share/mobile_manipulator_description
re=$(ros2 pkg prefix reachability)
xacro "$desc/urdf/mobile_manipulator.urdf.xacro" arm:="$arm" -o "$out/robot.urdf"
python3 "$re/lib/reachability/urdf_to_chain.py" "$out/robot.urdf" --base arm_base_link --tip tcp_link \
  -o "$out/${arm}_chain.yaml"
"$re/lib/reachability/rm4d_tool" build "$out/${arm}_chain.yaml" "$out/${arm}.rm4d" --samples "$samples" \
  --resolution 0.05 --extent 2.0 --zmin -1.8 --zmax 2.0
echo "map: $out/${arm}.rm4d"
