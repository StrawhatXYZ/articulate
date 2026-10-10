# Security

## Reporting a vulnerability

Please do not open a public issue for a security problem. Use GitHub's
**private vulnerability reporting**: the repository's **Security** tab →
**Report a vulnerability**. If that is not available, email the maintainer
listed in the packages' `package.xml` files.

Include what you found, how to reproduce it, and what it affects. We aim to
acknowledge a report within a week.

## What this project is, and is not

This is a research and learning lab that runs a **simulated** robot on your
own computer. It is not hardened for production robots, shared servers or
untrusted networks. Before driving real hardware, review the safety of the
whole system yourself: the safety check in `plan_check.py` validates plans
for this simulated table, not for a physical work cell.

## Things to keep in mind

- **The virtual desktop has no password.** The container's VNC and noVNC
  ports (5900, 6080) are published on `127.0.0.1` only, so only your own
  computer can reach them. Do not change `docker-compose.yml` to publish
  them on other interfaces.
- **The container runs as root**, as the official ROS images do. Do not
  mount sensitive host directories into it.
- **Anything that can reach the ROS graph can command the robot.** The
  instruction action (`/lab/instruct`) and the arm's controllers accept
  commands from any ROS 2 node on the same domain (`ROS_DOMAIN_ID=42`).
  Keep the container off untrusted networks.
- **The language model runs locally** through Ollama on your computer. The
  planner never sends instructions or camera images to an online service.
  The model's suggestions always pass through the deterministic safety
  check before the arm moves.
- **Model weights are downloaded** from Hugging Face and Ollama on first
  use. Pin the versions documented in the README if you need reproducible
  or audited builds.

## Supported versions

Only the latest release and the `main` branch receive fixes.
