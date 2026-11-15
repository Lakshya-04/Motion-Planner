# Engineering lessons from this codebase

Short, specific, and each tied to something that actually happened here.

## Design
- **Ports and adapters pay off immediately.** Planners, IK, collision and scene sit behind small interfaces in ROS-free cores. Swapping a backend became a configuration question, and the cores test in milliseconds.
- **One source of truth for robot structure.** A profile file names the arm, lift, base, links and joints; descriptions, planning config and nodes read it. Hard-coded names that survive in a few files are the first thing to break when a second robot is added.
- **Reuse before writing.** Adopt maintained open source behind an adapter. Check the licence file, not the badge, and check whether a dependency is statically linked into what you ship.

## Units and frames
- A distance metric that adds metres and radians is wrong in a different way for every robot. Weight the axes or avoid the metric.
- Decide the wrench convention once (who exerts force on whom, in which frame, at which point) and write it down. Then condition raw sensor data in a fixed order: sign, bias, payload gravity, lever-arm move.
- Plan against the pose the robot actually achieved, not the pose it was commanded to.
- Redundant arms: a pose goal samples random solutions. Seed the solver and prefer the solution closest to the current state.

## Contact and force control
- A compliant controller that accumulates an offset while blocked will release slowly when the reference retreats. Bound the offset and budget it in tracking-error thresholds.
- Simulators differ more than their names suggest: one held a 10 N contact to about 1 percent, the other was rigid and jumped to about 100 N for sub-millimetre penetration. A compliant tool tip or a different contact model is part of the simulator work, not an afterthought.
- The ground-truth wrench from a simulator is the best test oracle for the conditioning chain; publish it next to the raw sensor wrench.

## Testing
- Test the selector, not only the selected thing: a test per hardware mode caught a merge mistake that loaded the wrong plugin.
- A launch file that names a package that does not exist is a bug a ten-line test finds. Add that test.
- Clean-checkout runs find install-rule mistakes that a developer tree hides.
- Measure before claiming: figures in docs carry what was measured and when.

## Honest documentation
- Keep a status document with `done`, `partial` and `not done`, and evidence for each. Update it when things land.
- Comments that point to documents which are not in the repository are dead weight and a leak risk; state the rule in the comment instead.
- Mark unverified claims as unverified. It is cheaper than a wrong claim found later.
