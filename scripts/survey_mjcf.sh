#!/usr/bin/env bash
# Imports every MJCF (.xml) file under a folder and checks each against its
# original, printing MATCH, DIFFER or FAILED per file and a total.
#
#   scripts/survey_mjcf.sh project/tests/models
#   scripts/survey_mjcf.sh ~/mujoco_menagerie --verbose   # also print reports and differences
#   GODOT=/path/to/godot scripts/survey_mjcf.sh <folder>
#
# Build the extension first (scons). Files that are parts of other models
# (included by them) often can't be loaded on their own and show as FAILED.
set -uo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
godot="${GODOT:-godot}"
folder="${1:?usage: $0 <folder> [--verbose]}"
shift

if ! command -v "$godot" > /dev/null; then
    echo "error: Godot not found ('$godot'). Set GODOT=/path/to/godot." >&2
    exit 1
fi

# MuJoCo's own warnings and errors for each file go to stderr; only the survey's lines are kept.
"$godot" --headless --path "$repo_root/project" -s res://tools/mjcf_survey.gd -- "$(realpath "$folder")" "$@" 2> /dev/null \
    | grep -E '^(MATCH|DIFFER|FAILED|        |[0-9]+ files)'
