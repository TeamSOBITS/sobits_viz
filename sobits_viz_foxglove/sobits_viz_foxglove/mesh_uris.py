# Copyright 2026 Team SOBITS
# SPDX-License-Identifier: BSD-3-Clause
"""Rewrite the mesh URIs of a robot description for the Foxglove asset server."""

import re

# xacro expands $(find pkg) into the install path, which only a viewer on this
# machine can read; package:// is what the bridge's asset server resolves.
_INSTALLED = re.compile(r"file://[^\s\"'<>]*?/share/([^/\s\"'<>]+)/")


def rewrite_file_uris(description: str) -> tuple:
    """Return the description with installed file:// mesh URIs as package://."""
    return _INSTALLED.subn(r'package://\1/', description)
