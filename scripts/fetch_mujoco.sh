#!/usr/bin/env bash
# Downloads the MuJoCo release pinned in MUJOCO_VERSION and installs its headers,
# shared library and license into ./mujoco (or the directory given as $1).
#
#   scripts/fetch_mujoco.sh            # install into ./mujoco
#   scripts/fetch_mujoco.sh some/dir   # install elsewhere
#   MUJOCO_ARCHIVE=/path/to/mujoco-X-linux-x86_64.tar.gz scripts/fetch_mujoco.sh
#                                      # use an already-downloaded archive (still checksum-verified)
#
# To change version: update MUJOCO_VERSION, add the new archive's SHA-256 below,
# and update the libmujoco.so.<version> paths in project/bin/mujoco_physics.gdextension.
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
version="$(tr -d '[:space:]' < "$repo_root/MUJOCO_VERSION")"
dest="${1:-$repo_root/mujoco}"

case "$(uname -s)-$(uname -m)" in
    Linux-x86_64) archive="mujoco-${version}-linux-x86_64.tar.gz" ;;
    *)
        echo "error: no MuJoCo archive configured for $(uname -s)-$(uname -m)." >&2
        echo "Add the platform's archive name and checksum to $0." >&2
        exit 1
        ;;
esac

# SHA-256 of each supported release archive.
case "$archive" in
    mujoco-3.10.0-linux-x86_64.tar.gz) sha256="4b9ba10f1b43fe77768c2354a9789e863972715ceed009e508c55195b35a83c6" ;;
    *)
        echo "error: no checksum recorded for $archive; add it to $0." >&2
        exit 1
        ;;
esac

marker="$dest/.mujoco-version"
if [[ -e "$dest" ]]; then
    if [[ -f "$marker" && "$(cat "$marker")" == "$archive" ]]; then
        echo "MuJoCo $version already installed in $dest."
        exit 0
    fi
    echo "error: $dest already exists but was not installed by this script for $archive." >&2
    echo "Move or delete it, then run this script again." >&2
    exit 1
fi

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

if [[ -n "${MUJOCO_ARCHIVE:-}" ]]; then
    cp "$MUJOCO_ARCHIVE" "$tmp/$archive"
else
    url="https://github.com/google-deepmind/mujoco/releases/download/${version}/${archive}"
    echo "Downloading $url"
    curl --fail --location --silent --show-error --output "$tmp/$archive" "$url"
fi

echo "$sha256  $tmp/$archive" | sha256sum --check --quiet -

tar -xzf "$tmp/$archive" -C "$tmp"
src="$tmp/mujoco-${version}"

mkdir -p "$dest/include"
# include/ also holds headers for MuJoCo's bundled viewer (imgui, SDL2); only take MuJoCo's own.
cp -a "$src/include/mujoco" "$dest/include/"
cp -a "$src/lib" "$src/LICENSE" "$src/THIRD_PARTY_NOTICES" "$dest/"
echo "$archive" > "$marker"
echo "Installed MuJoCo $version into $dest."
