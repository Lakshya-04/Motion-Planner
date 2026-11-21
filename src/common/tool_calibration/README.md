---
type: Reference
title: tool_calibration
description: Package README for the ROS-free tool calibration record (JSON, validity, refusal check) -- the calibration maths lives in core_types.
tags: [calibration, tools, package-readme]
status: experimental
---

# tool_calibration

Header-only, ROS-free. One header, `tool_calibration/calibration_record.hpp`:

- `ToolCalibrationRecord`: TCP (`tcp_xyz`, `tcp_rpy`), payload mass and centre of gravity, F/T bias, the
  method that produced them (`pivot`, `plane_touch`, `fixture`, `cad`, ...), an ISO 8601 UTC date string,
  operator and robot id strings, residuals (RMS and maximum, tilt spread) and a validity block (maximum
  age in hours, mounting-event counter at calibration time).
- `toJson` / `fromJson` / `parseRecord` (nlohmann json). Reading rejects missing required fields and
  non-finite numbers.
- `ToolCalibrationRecord::isValid(now_epoch_s, current_mounting_events)` and `validityError(...)`.
- `requireValidCalibration(tool, record, now, mounting_events)`: a contact tool with a missing, expired
  or mounting-invalidated record is refused (throws); a non-contact tool passes without a record.

The maths that fills a record is in `core_types` (`pivotCalibration`, `planeTouchCalibration`,
`fitToolTilt`). Limits: simulation-tested only, no manager stage or file location convention yet, the
record is not yet compared with the profile's `tcp_xyz`. See `docs/calibration.md`.
