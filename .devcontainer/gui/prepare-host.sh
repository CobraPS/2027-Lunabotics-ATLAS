#!/usr/bin/env bash
# Run on the Linux desktop host, before Dev Containers creates/starts the GUI container.
# Prepares X11 authorization for traditional X11 or rootless XWayland sessions.
set +x
set -euo pipefail
umask 077

fail() {
    printf 'ATLAS Linux GUI: %s\n' "$*" >&2
    exit 1
}

[[ $(uname -s) == Linux ]] || fail 'Use the default devcontainer on non-Linux hosts.'
for tool in xauth sed stat mktemp chmod mv; do
    command -v "$tool" >/dev/null || fail "Missing host command '$tool'; install it before opening the Linux GUI configuration."
done
[[ ${DISPLAY:-} =~ ^(:|unix:)[0-9]+(\.[0-9]+)?$ ]] ||
    fail 'DISPLAY must identify a local X11/XWayland display (for example :0). Open VS Code from the desktop session.'
[[ ${XDG_RUNTIME_DIR:-} == /* && -d $XDG_RUNTIME_DIR && ! -L $XDG_RUNTIME_DIR && -O $XDG_RUNTIME_DIR ]] ||
    fail 'XDG_RUNTIME_DIR must be an absolute path to an existing directory owned by your desktop user.'
[[ $(stat -c '%a' -- "$XDG_RUNTIME_DIR") == 700 ]] ||
    fail 'XDG_RUNTIME_DIR must have permission mode 700.'
runtime=$(cd -- "$XDG_RUNTIME_DIR" && pwd -P)
repository=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)
[[ $runtime != "$repository" && $runtime != "$repository/"* ]] ||
    fail 'XDG_RUNTIME_DIR must be outside the repository; authentication data cannot be stored in the workspace.'
[[ ${ATLAS_GPU_GID-0} =~ ^[0-9]+$ ]] ||
    fail 'ATLAS_GPU_GID, when set, must be the numeric host GPU device group ID.'

authority_dir=$runtime/atlas-devcontainer-gui-$DISPLAY
if [[ -e $authority_dir || -L $authority_dir ]]; then
    [[ -d $authority_dir && ! -L $authority_dir && -O $authority_dir ]] ||
        fail 'Refusing an unsafe authority directory (must be owned by you, with no symlink).'
else
    mkdir -p -- "$authority_dir"
fi
chmod 700 -- "$authority_dir"

authority=$authority_dir/Xauthority
if [[ -e $authority || -L $authority ]]; then
    [[ -f $authority && ! -L $authority && -O $authority ]] ||
        fail 'Refusing an unsafe authority file (must be a regular file owned by you).'
fi

# Try to read X11 cookies from the host. If the X server has no cookies (rootless XWayland),
# create an empty Xauthority file - connections will work via the UNIX socket without auth.
entries=$(mktemp "$authority_dir/.entries.XXXXXX")
prepared=$(mktemp "$authority_dir/.authority.XXXXXX")
trap 'rm -f -- "$entries" "$prepared"' EXIT

# Never put cookie values in shell arguments, variables, or console output.
# xauth honors the desktop's XAUTHORITY, or its standard per-user default.
xauth nlist "$DISPLAY" >"$entries" 2>/dev/null ||
    fail 'Cannot read X11 authorization; check the host XAUTHORITY and desktop session.'

if [[ -s $entries ]]; then
    # Traditional X11 with cookies: extract and sanitize
    sed 's/^..../ffff/' "$entries" | xauth -f "$prepared" nmerge - >/dev/null 2>&1 ||
        fail 'Could not prepare the container X11 authority file.'
    [[ -s $prepared ]] || fail 'The prepared authority file is empty.'
    chmod 600 -- "$prepared"
    mv -f -- "$prepared" "$authority"
    printf 'ATLAS Linux GUI: X11 display authorization prepared (private, read-only container mount).\n'
else
    # No cookies found. Verify rootless XWayland works before accepting empty auth.
    # Try connecting with no XAUTHORITY - if it succeeds, empty auth is correct.
    if DISPLAY="$DISPLAY" XAUTHORITY=/dev/null timeout 3 xdpyinfo >/dev/null 2>&1; then
        : >"$authority"
        chmod 600 -- "$authority"
        printf 'ATLAS Linux GUI: XWayland rootless verified; no X11 cookies required.\n'
    else
        fail 'No X11 cookies found and rootless XWayland connection failed. Check host XAUTHORITY and XWayland configuration.'
    fi
fi