#!/usr/bin/env bash
# Runs the headless tests in project/tests against the built extension.
#
#   scripts/run_tests.sh                  # all tests, with `godot` from PATH
#   GODOT=/path/to/godot scripts/run_tests.sh
#   scripts/run_tests.sh test_simulation  # only the named tests
#
# Build the extension first (scons). A test passes when it exits 0 and prints
# "RESULT: 0 failure(s)" without any SCRIPT ERROR. Full output is shown for failures.
set -uo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
project="$repo_root/project"
godot="${GODOT:-godot}"

if ! command -v "$godot" > /dev/null; then
    echo "error: Godot not found ('$godot'). Set GODOT=/path/to/godot." >&2
    exit 1
fi

# A fresh checkout has no .godot/ folder, and Godot only loads the extension
# after the project has been imported once. On Godot 4.7.1 this first import
# crashes on exit with any GDExtension (a Godot bug), after the import itself has
# finished, so its exit code is ignored; a test that can't find the extension's
# classes fails with a SCRIPT ERROR instead. The braces keep bash from printing
# its own "Aborted" message for the crash.
{ ( "$godot" --headless --path "$project" --import ) > /dev/null 2>&1; } 2> /dev/null || true

if [[ $# -gt 0 ]]; then
    tests=("$@")
else
    tests=()
    for file in "$project"/tests/test_*.gd; do
        [[ -e "$file" ]] || continue
        name="$(basename "$file" .gd)"
        [[ "$name" == "test_base" ]] || tests+=("$name")
    done
fi

failed=()
for name in "${tests[@]}"; do
    # --fixed-fps makes every physics frame exactly 1/60 s, so results don't depend on machine speed.
    output="$(timeout 300 "$godot" --headless --fixed-fps 60 --path "$project" -s "res://tests/$name.gd" 2>&1)"
    code=$?
    summary="$(grep -E '^RESULT: ' <<< "$output" | tail -1)"
    if [[ $code -eq 0 && "$summary" == "RESULT: 0 failure(s)" ]] && ! grep -q "SCRIPT ERROR" <<< "$output"; then
        echo "PASS  $name ($(grep -c '^PASS ' <<< "$output") checks)"
    else
        echo "FAIL  $name (exit $code${summary:+, $summary})"
        echo "$output" | sed 's/^/      /'
        failed+=("$name")
    fi
done

echo
if [[ ${#failed[@]} -gt 0 ]]; then
    echo "${#failed[@]} of ${#tests[@]} test(s) failed: ${failed[*]}"
    exit 1
fi
echo "All ${#tests[@]} test(s) passed."
