"""Offline host-hook regression checks; uses synthetic cookies, never the desktop cookie."""

import os
from pathlib import Path
import shutil
import socket
import stat
import subprocess
import tempfile
import unittest


SCRIPT_X11 = Path(__file__).with_name('prepare-host.sh').resolve()
SCRIPT_WAYLAND = Path(__file__).with_name('prepare-host-wayland.sh').resolve()
BASH = shutil.which('bash')


class X11HostPreparationTests(unittest.TestCase):
    """Tests for X11/rootless-XWayland prepare-host.sh"""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='atlas-gui-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.runtime = self.root / 'runtime'
        self.runtime.mkdir(mode=0o700)
        self.source = self.root / 'host.xauth'
        self.source.touch(mode=0o600)
        self.cookie = '12' * 16
        self.environment = dict(os.environ, DISPLAY=':97', XAUTHORITY=str(self.source),
                                XDG_RUNTIME_DIR=str(self.runtime), ATLAS_GPU_GID='12345')
        self.set_cookie(self.cookie)
        self.directory = self.runtime / 'atlas-devcontainer-gui-:97'
        self.authority = self.directory / 'Xauthority'

    def set_cookie(self, value):
        subprocess.run(['xauth', '-f', str(self.source), 'add', ':97', '.', value],
                       check=True, capture_output=True)

    def run_hook(self, script=SCRIPT_X11, **overrides):
        result = subprocess.run([BASH, str(script)],
                                env=dict(self.environment, **overrides), cwd=self.root,
                                capture_output=True)
        self.assertNotIn(self.cookie.encode(), result.stdout + result.stderr)
        return result

    def test_private_cookie_and_atomic_refresh(self):
        """Traditional X11 with cookies: authority file contains sanitized cookies."""
        self.assertEqual(self.run_hook().returncode, 0)
        self.assertEqual(stat.S_IMODE(self.directory.stat().st_mode), 0o700)
        self.assertEqual(stat.S_IMODE(self.authority.stat().st_mode), 0o600)
        records = subprocess.check_output(['xauth', '-f', str(self.authority), 'nlist'])
        self.assertTrue(all(line.startswith(b'ffff') for line in records.splitlines()))
        self.assertIn(self.cookie.encode(), records)
        original = self.authority.read_bytes()
        directory_inode = self.directory.stat().st_ino
        self.set_cookie('34' * 16)
        self.assertEqual(self.run_hook().returncode, 0)
        self.assertNotEqual(self.authority.read_bytes(), original)
        self.assertEqual(self.directory.stat().st_ino, directory_inode)
        self.assertEqual(list(self.directory.iterdir()), [self.authority])

    def test_rootless_xwayland_empty_cookie_requires_verification(self):
        """Rootless XWayland (no cookies): requires xdpyinfo verification to succeed.

        In test env without real X server, xdpyinfo fails, so script correctly fails.
        On real rootless XWayland (niri), xdpyinfo succeeds and script creates empty auth.
        """
        self.source.write_bytes(b'')  # No cookies - simulates rootless XWayland
        # In test env without real X server, xdpyinfo fails, so script fails
        result = self.run_hook()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'rootless XWayland connection failed', result.stderr)
        # Authority file should not be created (or should be empty if created before fail)
        # The script fails before writing empty auth

    def test_missing_cookie_after_initial_success_fails(self):
        """If cookies disappear after first successful run with cookies, script fails verification.

        In test env without real X server, xdpyinfo fails, so script fails.
        On real system with rootless XWayland, it would succeed with empty auth.
        """
        # First run with cookies succeeds
        self.assertEqual(self.run_hook().returncode, 0)
        original = self.authority.read_bytes()
        self.assertGreater(len(original), 0)
        # Then cookies disappear
        self.source.write_bytes(b'')
        result = self.run_hook()
        # In test env without X server, xdpyinfo verification fails
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'rootless XWayland connection failed', result.stderr)
        # Original authority should be untouched (script fails before writing)
        self.assertEqual(self.authority.read_bytes(), original)

    def test_invalid_environment_fails_clearly(self):
        for change, message in [({'DISPLAY': ''}, b'DISPLAY'),
                                ({'DISPLAY': 'localhost:10.0'}, b'local X11'),
                                ({'XDG_RUNTIME_DIR': ''}, b'XDG_RUNTIME_DIR'),
                                ({'XDG_RUNTIME_DIR': 'runtime'}, b'XDG_RUNTIME_DIR'),
                                ({'ATLAS_GPU_GID': ''}, b'numeric'),
                                ({'ATLAS_GPU_GID': 'render'}, b'numeric')]:
            with self.subTest(change=change):
                result = self.run_hook(**change)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)

    def test_missing_xauth_fails_clearly(self):
        binaries = self.root / 'bin'
        binaries.mkdir()
        (binaries / 'uname').symlink_to(shutil.which('uname'))
        result = self.run_hook(PATH=str(binaries))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b"Missing host command 'xauth'", result.stderr)

    def test_insecure_runtime_and_symlinks_are_rejected(self):
        """Security checks still reject insecure runtime dir and symlinks."""
        self.runtime.chmod(0o755)
        self.assertNotEqual(self.run_hook().returncode, 0)
        self.runtime.chmod(0o700)
        link = self.root / 'runtime-link'
        link.symlink_to(self.runtime, target_is_directory=True)
        self.assertNotEqual(self.run_hook(XDG_RUNTIME_DIR=str(link)).returncode, 0)
        self.directory.symlink_to(self.root, target_is_directory=True)
        self.assertNotEqual(self.run_hook().returncode, 0)
        self.directory.unlink()
        self.directory.mkdir(mode=0o700)
        self.authority.symlink_to(self.source)
        self.assertNotEqual(self.run_hook().returncode, 0)

    def test_runtime_inside_workspace_is_rejected_before_writing_cookies(self):
        fake_gui = self.root / 'workspace' / '.devcontainer' / 'gui'
        fake_gui.mkdir(parents=True)
        copied = fake_gui / 'prepare-host.sh'
        shutil.copyfile(SCRIPT_X11, copied)
        runtime = self.root / 'workspace' / 'runtime'
        runtime.mkdir(mode=0o700)
        self.assertNotEqual(self.run_hook(script=copied, XDG_RUNTIME_DIR=str(runtime)).returncode, 0)
        self.assertEqual(list(runtime.iterdir()), [])


class WaylandHostPreparationTests(unittest.TestCase):
    """Tests for Wayland prepare-host-wayland.sh"""

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='atlas-gui-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.runtime = self.root / 'runtime'
        self.runtime.mkdir(mode=0o700)
        self.source = self.root / 'host.xauth'
        self.source.touch(mode=0o600)
        self.cookie = '12' * 16
        # Create a real Unix domain socket for Wayland
        self.wayland_sock = self.runtime / 'wayland-0'
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.bind(str(self.wayland_sock))
        self.sock.listen(1)
        self.addCleanup(self.sock.close)

        self.environment = dict(os.environ, DISPLAY=':97', XAUTHORITY=str(self.source),
                                XDG_RUNTIME_DIR=str(self.runtime), ATLAS_GPU_GID='12345',
                                WAYLAND_DISPLAY='wayland-0')
        self.set_cookie(self.cookie)
        self.directory = self.runtime / 'atlas-devcontainer-gui-:97'
        self.authority = self.directory / 'Xauthority'

    def set_cookie(self, value):
        subprocess.run(['xauth', '-f', str(self.source), 'add', ':97', '.', value],
                       check=True, capture_output=True)

    def run_hook(self, **overrides):
        result = subprocess.run([BASH, str(SCRIPT_WAYLAND)],
                                env=dict(self.environment, **overrides), cwd=self.root,
                                capture_output=True)
        self.assertNotIn(self.cookie.encode(), result.stdout + result.stderr)
        return result

    def test_wayland_socket_required(self):
        """Script fails if WAYLAND_DISPLAY not set or socket missing."""
        # No WAYLAND_DISPLAY
        result = self.run_hook(WAYLAND_DISPLAY='')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'WAYLAND_DISPLAY must be set', result.stderr)

        # WAYLAND_DISPLAY set but socket missing
        result = self.run_hook(WAYLAND_DISPLAY='wayland-1')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'Wayland socket', result.stderr)

    def test_wayland_with_x11_cookies_preserves_auth(self):
        """KDE Wayland with authenticated XWayland: preserves X11 cookies."""
        self.assertEqual(self.run_hook().returncode, 0)
        self.assertTrue(self.authority.exists())
        self.assertGreater(self.authority.stat().st_size, 0)
        records = subprocess.check_output(['xauth', '-f', str(self.authority), 'nlist'])
        self.assertTrue(all(line.startswith(b'ffff') for line in records.splitlines()))
        self.assertIn(self.cookie.encode(), records)
        self.assertIn(b'X11 display authorization prepared', self.run_hook().stdout)

    def test_wayland_rootless_xwayland_empty_cookie_requires_verification(self):
        """Wayland with rootless XWayland (no cookies): requires xdpyinfo verification.

        In test env without real X server, xdpyinfo fails, so script correctly fails.
        On real rootless XWayland, xdpyinfo succeeds and script creates empty auth.
        """
        self.source.write_bytes(b'')
        result = self.run_hook()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(b'rootless XWayland connection failed', result.stderr)

    def test_wayland_invalid_runtime_dir_rejected(self):
        """Security checks reject insecure runtime dir."""
        self.runtime.chmod(0o755)
        self.assertNotEqual(self.run_hook().returncode, 0)

    def test_wayland_symlink_rejected(self):
        """Security checks reject symlinks."""
        self.directory.symlink_to(self.root, target_is_directory=True)
        self.assertNotEqual(self.run_hook().returncode, 0)
        self.directory.unlink()
        self.directory.mkdir(mode=0o700)
        self.authority.symlink_to(self.source)
        self.assertNotEqual(self.run_hook().returncode, 0)

    def test_wayland_outputs_socket_path(self):
        """Script outputs the Wayland socket path for verification."""
        result = self.run_hook()
        self.assertEqual(result.returncode, 0)
        expected_sock = str(self.runtime / 'wayland-0')
        self.assertIn(expected_sock.encode(), result.stdout)


if __name__ == '__main__':
    unittest.main()