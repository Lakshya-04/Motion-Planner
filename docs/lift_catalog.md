# Lift catalog

Telescopic and prismatic lifts of well-known mobile manipulators and a few commercial linear columns,
written as profiles (data only). Every lift is a primitive-shape `telescopic` lift (one master prismatic
`lift_joint` plus 0 to 3 visual mimic stages, see [robot_description.md](robot_description.md#lift-variants)): no vendor
meshes, no new lift macro. The numbers are the published ones where a source is given; the model is a
generic column with those stroke, speed and payload figures, not a copy of the vendor's mechanism.

Status: **verified** = read from the cited page or datasheet on the date below; **partial** = some numbers
verified, the rest approximate; **approximate** = not found in a primary source, estimated. Date checked:
2026-11-22. Specifications change without notice: check them against your unit.

| Robot | Profile (arm + base + lift) | Stroke | Stages | Max speed | Payload | Source | Status |
|---|---|---|---|---|---|---|---|
| PAL TIAGo torso | `fr3_tiago_style_torso` (FR3, `tiago_style`) | 0.35 m | not published (0 = one carriage) | 0.05 m/s | not published for the lift (5 kg laptop tray only) | [PAL docs, TIAGo torso](https://docs.pal-robotics.com/edge/hardware/tiago/torso) | verified (stroke, speed); stages and payload not published |
| Fetch torso | `generic_6dof_fetch_style_torso` (generic 6-DOF, `fetch_style`) | 0.386 m in the profile; 0.40 m URDF hard limit published; 0.395 m from the spec-sheet heights | not published (0) | 0.1 m/s joint limit published, profile uses a derated 0.05 | 450 N force limit published, no payload | [Fetch docs, robot hardware](https://docs.fetchrobotics.com/robot_hardware.html) (limits as quoted by a search result; the host did not resolve when fetched), heights 1,096 to 1,491 mm in the [spec table](https://www.wevolver.com/specs/fetch.mobile.manipulator) | approximate: 0.386 m not found in a primary source |
| Willow Garage PR2 spine | `generic_6dof_pr2_spine_lift` (generic 6-DOF, `generic_base`) | 0.31 m | 0 (linear rail, lead screw and gas spring; one carriage) | 0.013 m/s (joint velocity limit) | not published (effort limit 10,000 N is a controller limit) | [PR2 user manual r321, sections 6.2, 8.1, 8.3](https://www.clearpathrobotics.com/assets/downloads/pr2/pr2_manual_r321.pdf) | verified (stroke, speed); mount height approximate |
| Toyota HSR | `generic_6dof_hsr_torso_lift` (generic 6-DOF, `tiago_style`) | 0.69 m (shoulder height 340 to 1,030 mm; body height 1,005 to 1,350 mm, a 0.345 m torso span, is the other half) | 0 modelled (one carriage; the real torso and arm lift are two sections) | not found (0.05 m/s assumed) | not found (the hand holds 1.2 kg) | [Toyota HSR spec sheet](https://global.toyota/detail/8709541) | partial: stroke derived from published heights; speed and mount height approximate |
| Hello Robot Stretch (RE1 spec sheet) | `generic_6dof_stretch_mast_lift` (generic 6-DOF, `turtlebot3_waffle`) | 1.10 m | 0 (carriage on a fixed mast) | 0.2 m/s | 1.5 kg (arm, no gripper) | [Stretch RE1 spec sheet, 7.2020](https://www.therobotreport.com/wp-content/uploads/2020/07/Stretch-RE1-Specification-7.2020.pdf) | verified for RE1; Stretch 3 hardware guide does not list the lift range, Stretch 4 lists 1.20 m and 0.5 m/s ([datasheet](https://hello-robot.com/wp-content/uploads/2026/05/HelloRobot-DataSheet-Stretch-4-Rev5_AsLaunched.pdf)), not modelled |
| Vention telescopic lift column (commercial) | `fr3_column870_lift` (FR3, `mpo_500`) | 0.87 m total travel (830 mm compressed, 1,700 mm extended) | not published (2 assumed; two lead screws) | 0.075 m/s nominal (up to 0.1 m/s unloaded) | 100 kg standard, 225 kg maximum | [Vention docs, telescopic lift column](https://docs.vention.com/docs/en/telescopic-lift-column) | verified (stroke, speed, payload, heights); stages assumed |
| TiMOTION TL13S lifting column (commercial) | `xarm7_column500_lift` (xArm 7, `scout_mini`) | 0.50 m | 2 | 0.03 m/s at full load (0.038 unloaded) | 700 N push load (about 70 kg), push only | [TL13S datasheet](https://pre.timotion.com/_upload/files/DataSheet_TL13S-D_TN.pdf) (645 mm retracted, 70 x 70 mm outer tube, 24 V DC) | verified |

To pick one for a task, see [Choosing a lift profile](lift.md#choosing-a-lift-profile).

Candidates dropped for lack of a published figure: Robotnik RB-KAIROS (the 2024 datasheets describe a fixed-height
platform, no lift found), Festo and Phoenix Mecano columns (no stroke, speed and payload page found), Clearpath
lift options (none found). Other industrial columns seen but not added: Jiecang JC35EM2 (2 stages, 600 mm, 6,000 N, speed
not found) and Movetec MAC 210 (700 mm, 6,000 to 8,000 N, 27.1 mm/s at 2,000 N), from distributor listings only.

## Notes

- The profile `lift:` section holds `stroke_m`, `stages`, `max_velocity_mps`, `max_acceleration_mps2`
  (a planning default, none of the sources publish it) and `mount_height_m`. `payload_kg` is a note only and is not
  read by the model. Mount heights are estimates from the published overall heights and are not in the sources.
- The arms are the generic or supported ones (`generic_6dof`, `fr3`, `xarm7`), not the robots' own arms, and the bases
  are the existing ones nearest in size. A catalog profile says "this lift with this arm", not "this robot".
- The [telescopic](../src/description/mobile_manipulator_description/urdf/lift/telescopic_lift.xacro) macro models stages
  as equal mimic sections of one tube; it cannot model unequal stage travel, a counterweight or a gas spring (PR2).
  Gap: no lift type models the Stretch mast or the HSR's two independent joints (torso and arm lift); both are
  represented as one prismatic joint of the combined stroke.
- `test/test_lift_catalog.py` of `mobile_manipulator_description` reads the profile names and strokes out of this table and
  checks them against the profile files.
