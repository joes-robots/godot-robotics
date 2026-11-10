#!/usr/bin/env bash
# Downloads the official Godot editor binary used by CI to run the tests, and
# prints its path. The version is the extension's compatibility_minimum, so CI
# also checks that nothing newer than it is required.
#
#   GODOT="$(scripts/fetch_godot.sh some/dir)"
#   GODOT_ARCHIVE=/path/to/Godot_vX_linux.x86_64.zip scripts/fetch_godot.sh some/dir
#                       # use an already-downloaded archive (still checksum-verified)
#
# To change version: update `version` and the archive's SHA-256 below (Godot
# publishes SHA512-SUMS.txt with each release to check a download against).
set -euo pipefail

version="4.4.1-stable"
dest="${1:?usage: $0 <directory>}"

case "$(uname -s)-$(uname -m)" in
    Linux-x86_64) archive="Godot_v${version}_linux.x86_64.zip" ;;
    *)
        echo "error: no Godot archive configured for $(uname -s)-$(uname -m)." >&2
        exit 1
        ;;
esac

case "$archive" in
    Godot_v4.4.1-stable_linux.x86_64.zip) sha256="d6e382fb531019f85630c1f485a561a0d20c4a2344b6c3847735cfee7da812aa" ;;
    *)
        echo "error: no checksum recorded for $archive; add it to $0." >&2
        exit 1
        ;;
esac

binary="$dest/${archive%.zip}"
if [[ -x "$binary" ]]; then
    echo "$binary"
    exit 0
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

if [[ -n "${GODOT_ARCHIVE:-}" ]]; then
    cp "$GODOT_ARCHIVE" "$tmp/$archive"
else
    url="https://github.com/godotengine/godot/releases/download/${version}/${archive}"
    echo "Downloading $url" >&2
    curl --fail --location --silent --show-error --output "$tmp/$archive" "$url"
fi
echo "$sha256  $tmp/$archive" | sha256sum --check --quiet - >&2

mkdir -p "$dest"
unzip -q -o "$tmp/$archive" -d "$dest"
chmod +x "$binary"
echo "$binary"
