# Contributing to sobits_viz

Working rules for anyone adding code to this repo. Every PR is expected to
follow these; reviewers will send you back here.

## Node files are wiring only

A `*_node.py` / `*.cpp` node owns: parameter reading, pub/sub/timer/service
creation, and callback dispatch. Every non-trivial computation lives in a
plain, dependency-free module that takes values and returns values.

Test: *can this logic be unit-tested without `rclpy.init()` (or, in C++,
without constructing an `rclcpp::Node`)?* If no, extract it.

## Comments: 2 lines max, why only

A comment is at most 2 lines and explains *why*, never *what*. If a comment
merely restates the code, the correct comment is no comment — delete it,
don't compress it.

## Naming

- Packages: `sobits_viz_<viewer>` (e.g. `sobits_viz_rerun`).
- Per-robot config: `config/<robot>/<robot>{.robot.yaml,.yaml,.rbl}`, one
  folder per robot.

## Commits

One commit per discrete fix or implementation step — no batching unrelated
changes. Subject line in the imperative mood (`Add ...`, `Fix ...`, not
`Added`/`Fixes`).
