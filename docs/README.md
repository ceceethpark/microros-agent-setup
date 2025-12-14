# ROS2 테스트 프로젝트 문서

ROS2 관련 테스트 및 개발 내용을 기록하는 문서입니다.

## 문서 구조
## 프로젝트 개요

# Documentation Index

Short guide and links for this project's documentation.

- **Overview:** [1_yb_robotCar_개요및개발.md](1_yb_robotCar_개요및개발.md)
- **micro-ROS / Agent:** [2_microROS.md](2_microROS.md)
- **Setup & Installation:** [3_setup.md](3_setup.md)
- **Notes & Analysis:** [4_notes.md](4_notes.md)
- **Test Logs / Repro:** [5_test_log.md](5_test_log.md)

Usage:
- Read `1_yb_robotCar_개요및개발.md` for system context.
- Follow `3_setup.md` to install dependencies and configure systemd.
- Use `2_microROS.md` for quick run/debug commands.
- Capture logs with `scripts/collect_logs.sh` when debugging.

```
- Use `scripts/collect_logs.sh` (or `docs/scripts/collect_logs.sh` if present) to capture Agent verbose logs, RTPS pcaps, and serial dumps for analysis.

If you'd like, I can now (a) remove the numeric prefixes from filenames, (b) split or merge any of the above files, or (c) convert these into a single index page — tell me which you prefer.
