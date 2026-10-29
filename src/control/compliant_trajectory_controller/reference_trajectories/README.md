# Reference trajectories (this branch only, not intended for main)

CSVs captured by `manipulation_manager`'s `record_joint_trajectory.py` (reused as-is --
generic `/joint_states`-shaped recorder, no changes needed) against a topic here, not a
FollowJointTrajectory replay target -- these are before/after snapshots of `jtac_compliance_demo`
behavior, kept so a future tuning change (e.g. per-joint redundancy weighting) can be compared
against a known "what it did before" baseline instead of trusting memory of what RViz looked like.

## wall_ceiling_demo_{actual,reference}_pre_redundancy_weighting_20260821.csv

Recorded 2026-08-21, one full `pose_set:=wall_ceiling` schedule cycle (~52.5s, 60s captured with
margin) via:

```
ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py \
  pose_set:=wall_ceiling show_rviz:=true

# concurrently:
ros2 run manipulation_manager record_joint_trajectory.py \
  --output-path wall_ceiling_demo_actual_pre_redundancy_weighting_20260821.csv \
  --joint-state-topic /joint_states --max-duration-sec 60
ros2 run manipulation_manager record_joint_trajectory.py \
  --output-path wall_ceiling_demo_reference_pre_redundancy_weighting_20260821.csv \
  --joint-state-topic /ghost/joint_states --max-duration-sec 60
```

`actual` = `/joint_states` (the compliance-shifted pose `AdmittanceRule` actually produced);
`reference` = `/ghost/joint_states` (the fixed pose each phase held as its spring's zero point --
constant within a phase, stepping at each `WallCeilingPhaseKind::kMoveToPose`). The gap between
the two at any timestamp is the live compliance offset.

State captured at the moment this was recorded, both since-fixed in the same session:

- `kHomeWallPose`/`kCeilingPose`: `joint1 = +45deg` (the robot's actual front, per `attach_rpy`'s
  own +45deg `base_serum` mount) -- lands the tool's local Z on pure base -Y at the wall pose,
  pure +Z at the ceiling pose (see those constants' own doc comments in
  `jtac_compliance_demo_node.cpp`).
- `admittance.mass`/`stiffness`/`damping_ratio`: X/Y triple reduced to exactly match Z
  (`mass=2.0, damping_ratio=6.0, stiffness=300.0` on all three translational axes) -- was
  `mass=4.0/damping_ratio=5.59/stiffness=1000.0` (X/Y) vs `2.0/6.0/300.0` (Z), a real "arm 3.3x
  stiffer than lift" asymmetry.

**Live-verified lift-share numbers at this state** (fraction of the resulting Cartesian motion the
lift alone accounts for, at steady hold -- see `jtac_compliance_demo_node.cpp`'s
`liftShareAlongDirection()`): WALL +Z hold 0%, WALL +X hold 66%, WALL +Y hold 0%, CEILING +Z hold
96.5%, CEILING +X hold 0%. This split is pure Jacobian geometry (which joint's column projects
onto the push direction) and is unaffected by the mass/stiffness/damping_ratio values above --
changing those changes offset *magnitude*, not this *split*. That split having no tuning dial is
exactly the gap the redundancy-weighting feature (per-joint weighted pseudo-inverse) this baseline
precedes is meant to close.

Not intended for `replay_joint_trajectory.py` -- this is a diagnostic demo cycling in place, not
a coverage mission with a meaningful start/end to reproduce.

## wall_ceiling_demo_actual_redundancy_weighted_default_20260821.csv

Recorded 2026-08-21, same schedule/state as above plus `enable_redundancy_weighting:=true` (no
`redundancy_weights` override -- the chosen default, `[10, 1, 1, 1, 1, 1, 1]`, lift weighted 10x
each arm joint):

```
ros2 launch compliant_trajectory_controller jtac_compliance_demo.launch.py \
  pose_set:=wall_ceiling show_rviz:=true enable_redundancy_weighting:=true

ros2 run manipulation_manager record_joint_trajectory.py \
  --output-path wall_ceiling_demo_actual_redundancy_weighted_default_20260821.csv \
  --joint-state-topic /joint_states --max-duration-sec 60
```

See `force_control_backend_admittance_vs_diffik.md`'s T11 section for the full mechanism, the
literature it's drawn from, and the verified range this default sits inside (1.2% lift share at an
extreme 1000:1 arm-biased ratio, ~93-96% at the natural 1:1 unweighted split, 100% at the opposite
1:1000 extreme). At this chosen default: CEILING +Z hold lift share **55.8%** (down from the
unweighted ~93-96%, per the "arm more compliant than the lift" design choice), WALL +X hold
**14.8%** (down from 66%); WALL +Z and CEILING +X both stayed at 0%, same as unweighted -- the
lift's Jacobian column is nearly orthogonal to those two push directions regardless of weight, a
genuine limit of the mechanism, not a bug (no reference recording for this one: `/ghost/
joint_states` is identical to the unweighted case, since redundancy weighting only changes how
`current_` responds, not the reference each phase holds).
