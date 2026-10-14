# tf-route-audit

Offline TF topology and time-window diagnostics from exported ROS TransformStamped records.

**English** · [简体中文](README.zh-CN.md) · [Demand evidence](docs/EVIDENCE.md) · [License](LICENSE)

Independent, experimental utility maintained by **Afloat16**. Original implementation; no upstream endorsement is implied.

## Install

Python 3.10+.

```bash
git clone https://github.com/Afloat16/tf-route-audit.git
cd tf-route-audit
python -m venv .venv
# Activate the virtual environment for your OS, then:
python -m pip install -e '.[dev]'
```

The core package uses the Python standard library only. No GPU, ROS runtime, API key, hosted service or model weights are required for the core offline workflow.

## First run

```bash
tf-route-audit examples/transforms.json --query map base_link --at-ns 150
tf-route-audit robot.urdf --require base_link --query base_link camera_link
tf-route-audit capture.json --strict --output tf-report.json
```

## Inputs and checks

Native URDF, a JSON transform array, `{transforms:[...]}`, or `{messages:[...]}` wrappers containing ROS TransformStamped objects. Normalized records use `{parent,child,stamp_ns,static,rotation:[x,y,z,w],translation:[x,y,z]}`. Rotation/translation may be omitted for topology-only exports. A message `topic` equal to /tf_static marks its transforms static. Native ROS seconds/nanoseconds are kept as integer nanoseconds.

Checks cycles, multiple parents in aggregated evidence, static/dynamic conflicts, disconnected components, required frames, quaternion validity and dynamic timestamp regressions. `--query A B` reports topology routes in either direction; `--at-ns` checks dynamic observed ranges and treats static edges as timeless.

## Boundaries

Does not implement a tf2 cache, compose/interpolate poses, discover live authority or evaluate xacro. Inside-range queries can still cross sampling gaps. Movable URDF joints have no time evidence and cannot pass timed queries. JSON is an explicit export contract, not a claim that every ROS CLI emits this format. Use tf2_echo/view_frames/tf2_monitor for live validation. Limits: 64 MiB JSON, 16 MiB URDF, 1,000,000 transforms and 10,000 frames.

## Output and development

Audit commands exit 0 for no error findings, 1 for errors (also warnings with --strict), and 2 for malformed input/report-write failure. --output creates a new JSON file exclusively; existing files are never overwritten. Detailed audit findings are capped at 200 while total severity counts remain accurate.

```bash
python -m unittest discover -s tests -v
python -m compileall -q src tests
python -m build
```

The configured CI matrix covers Ubuntu, Windows and macOS with Python 3.10/3.13; inspect Actions for actual results. Fixtures are synthetic. No private captures or upstream user attachments are redistributed.

Report problems with command/version, expected/actual result and a minimal synthetic reproduction. Keep source attribution and compatible licensing. MIT license, copyright Afloat16.


## Related projects

[Browse Afloat16's project guide](https://github.com/Afloat16/research-to-fix/blob/main/docs/PROJECTS.md) for robotics diagnostics, model interfaces, 3D geometry and engineering workflows.
