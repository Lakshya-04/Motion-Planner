# Security policy

## Reporting a vulnerability

Please report security problems privately through GitHub's **private vulnerability reporting**
(the repository's *Security* tab, *Report a vulnerability*), not in a public issue or pull request.
Include what you found, the affected package and commit, and steps to reproduce. You will get an
acknowledgement, and a fix or a mitigation note in the repository once the problem is confirmed.

## Scope and expectations

This is research software for robot motion planning and control. It has **not** been run on hardware
and carries no safety certification: do not rely on it to protect people or equipment (the force limits,
tool-actuation safety and stand checks documented here are evaluated in simulation only, see
[docs/ROADMAP.md](docs/ROADMAP.md)). Reports about a way to make it command unsafe motion, leak
data, or execute code through its ROS interfaces, launch files, scripts or Docker images are in scope.
Only the latest commit on `main` is supported; there are no release branches.

## Third-party software

Dependencies are consumed upstream and unpatched ([THIRD_PARTY.md](THIRD_PARTY.md)); report their
vulnerabilities to their own projects.
