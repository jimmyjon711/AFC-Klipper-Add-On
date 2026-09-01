"""Regression tests for issue #853.

Two related installer bugs:

1. ``afc_path`` was hardcoded to ``$HOME/AFC-Klipper-Add-On`` in both
   ``install-afc.sh`` and ``update-afc.sh``. Cloning the repo anywhere else
   (e.g. ``/usr/data`` on Creality K1 machines, where cloning into ``/root``
   can fill the tiny rootfs) made the installer ignore the real checkout,
   print a bogus "installed via ZIP file" warning, and then fail with
   ``cp: can't stat '.../config/AFC.cfg'``. It must resolve from the
   script's own location (``SCRIPT_DIR``) instead.

2. ``install-afc.sh`` documented a ``-m`` flag (Moonraker config path) but
   unconditionally overwrote it with ``$printer_config_dir/moonraker.conf``.

3. The Moonraker ``[update_manager afc-software]`` block hardcoded
   ``path: ~/AFC-Klipper-Add-On`` instead of the resolved add-on path.
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
ANSI = re.compile(r"\x1b\[[0-9;]*m")

# Files/dirs the installer needs to reach the main menu / run an update.
_CHECKOUT_ITEMS = ("install-afc.sh", "update-afc.sh", "include", "config", "templates", "extras")


def _make_checkout(dest: Path) -> Path:
    """Copy the add-on tree to ``dest`` with a ``.git`` dir so it reads as a
    real git checkout."""
    dest.mkdir(parents=True)
    for item in _CHECKOUT_ITEMS:
        src = REPO_ROOT / item
        dst = dest / item
        if src.is_dir():
            shutil.copytree(src, dst)
        else:
            shutil.copy2(src, dst)
    (dest / ".git").mkdir()
    return dest


@pytest.fixture
def fake_checkout(tmp_path: Path) -> Path:
    """A checkout at a path *outside* ``$HOME``."""
    return _make_checkout(tmp_path / "opt" / "AFC-Klipper-Add-On")


def _make_zip_extract(dest: Path) -> Path:
    """Like ``_make_checkout`` but with no ``.git`` dir, mimicking an
    extracted release archive."""
    checkout = _make_checkout(dest)
    shutil.rmtree(checkout / ".git")
    return checkout


def _run_installer(checkout: Path, home: Path, *args: str, stdin: str = "Q\n") -> tuple[int, str]:
    (home / "printer_data" / "config").mkdir(parents=True, exist_ok=True)
    (home / "klipper" / "klippy" / "extras").mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env["HOME"] = str(home)
    env["LC_ALL"] = "C"
    env["TERM"] = "xterm"  # the scripts call `clear`, which errors under `set -e` with no TERM
    proc = subprocess.run(
        [
            "bash",
            "install-afc.sh",
            "-t",  # test_mode: skip python-version check and the git clone/restart
            "-p", str(home / "printer_data" / "config"),
            "-k", str(home / "klipper"),
            "-y", str(home / "klippy-env" / "bin"),
            *args,
        ],
        cwd=checkout,
        env=env,
        input=stdin,
        capture_output=True,
        text=True,
        timeout=60,
    )
    return proc.returncode, ANSI.sub("", proc.stdout + proc.stderr)


def test_afc_path_follows_script_location_not_home(fake_checkout: Path, tmp_path: Path):
    # Running from a real checkout that is NOT at $HOME/AFC-Klipper-Add-On must
    # not be mistaken for a ZIP install, and must exit cleanly.
    rc, out = _run_installer(fake_checkout, tmp_path / "home")
    assert "installed via ZIP" not in out, out
    assert rc == 0, out


def test_canonical_home_checkout_still_works(tmp_path: Path):
    # The common case (repo cloned at ~/AFC-Klipper-Add-On) is unchanged:
    # SCRIPT_DIR resolves to the same place $HOME/AFC-Klipper-Add-On used to,
    # so there is no ZIP warning and the installer still reaches the menu.
    home = tmp_path / "home"
    checkout = _make_checkout(home / "AFC-Klipper-Add-On")
    rc, out = _run_installer(checkout, home)
    assert "installed via ZIP" not in out, out
    assert rc == 0, out
    default_moonraker = home / "printer_data" / "config" / "moonraker.conf"
    assert f"Moonraker Config File    : {default_moonraker}" in out, out


def test_zip_extract_outside_home_is_detected_and_usable(tmp_path: Path):
    # No .git dir (extracted archive). The installer must still recognise it as
    # a ZIP install and operate on the extracted location, wherever it is,
    # rather than a hardcoded ~/AFC-Klipper-Add-On.
    home = tmp_path / "home"
    extract = _make_zip_extract(tmp_path / "downloads" / "AFC-Klipper-Add-On-main")
    # First newline answers check_for_zip_install's "Press Enter"; Q exits the menu.
    rc, out = _run_installer(extract, home, stdin="\nQ\n")
    assert "installed via ZIP file" in out, out
    assert "config/AFC.cfg" not in out, out  # i.e. no "can't stat" failure
    assert rc == 0, out


def _run_update_from_zip_extract(tmp_path: Path, *extra_args: str) -> tuple[subprocess.CompletedProcess, Path, Path]:
    home = tmp_path / "home"
    cfg = home / "printer_data" / "config"
    (cfg / "AFC" / "macros").mkdir(parents=True)
    # A realistic post-install AFC config so remove_velocity() has cfgs to scan.
    (cfg / "AFC" / "AFC.cfg").write_text("[AFC_buffer Turtle_1]\nvelocity: 100\n")
    klipper_extras = home / "klipper" / "klippy" / "extras"
    klipper_extras.mkdir(parents=True)
    venv = home / "venv"
    venv.mkdir(parents=True)
    (venv / "python").symlink_to(shutil.which("python3") or sys.executable)
    extract = _make_zip_extract(tmp_path / "downloads" / "AFC-Klipper-Add-On-main")
    env = os.environ.copy()
    env["HOME"] = str(home)
    env["LC_ALL"] = "C"
    env["TERM"] = "xterm"  # the scripts call `clear`, which errors under `set -e` with no TERM
    proc = subprocess.run(
        ["bash", "update-afc.sh", *extra_args, "-p", str(cfg),
         "-k", str(home / "klipper"), "-y", str(venv)],
        # U -> update; Enter past the "manually updated?" notice; n -> keep
        # macros; Q -> quit.
        cwd=extract, env=env, input="U\n\nn\nQ\n",
        capture_output=True, text=True, timeout=60,
    )
    return proc, extract, klipper_extras


@pytest.mark.parametrize("test_mode_flag", [[], ["-t"]], ids=["normal", "test-mode"])
def test_update_from_zip_extract_runs_normal_update_sequence(tmp_path: Path, test_mode_flag):
    # Someone updates a ZIP install by extracting a fresh archive over the
    # directory, then runs update-afc.sh from it. It must NOT try to `git clone`
    # into the non-empty dir, and no git command must run against the non-git
    # dir (get_git_version used to `cd` in and `git rev-parse`, failing with
    # "fatal: not a git repository"). The normal update sequence still runs, so
    # link_extensions symlinks the extras into Klipper.
    proc, extract, klipper_extras = _run_update_from_zip_extract(tmp_path, *test_mode_flag)
    out = ANSI.sub("", proc.stdout + proc.stderr)
    assert proc.returncode == 0, out
    assert "already exists and is not an empty directory" not in out, out
    assert "not a git repository" not in out, out
    linked = klipper_extras / "AFC.py"
    assert linked.is_symlink(), f"link_extensions did not run\n{out}"
    assert linked.resolve() == (extract / "extras" / "AFC.py").resolve()


def test_m_flag_is_honoured(fake_checkout: Path, tmp_path: Path):
    custom = tmp_path / "elsewhere" / "moonraker.conf"
    rc, out = _run_installer(fake_checkout, tmp_path / "home", "-m", str(custom))
    assert f"Moonraker Config File    : {custom}" in out, out


def test_m_flag_defaults_to_printer_config_dir(fake_checkout: Path, tmp_path: Path):
    home = tmp_path / "home"
    rc, out = _run_installer(fake_checkout, home)
    expected = home / "printer_data" / "config" / "moonraker.conf"
    assert f"Moonraker Config File    : {expected}" in out, out


def _run_moonraker_update(afc_path: str, home: str) -> str:
    # Source constants + update_commands, stub out restart_service, then let
    # update_moonraker_config write the [update_manager afc-software] block.
    script = f"""
set -u
REPO="{REPO_ROOT}"
source "$REPO/include/constants.sh"
source "$REPO/include/update_commands.sh"
restart_service() {{ :; }}
export HOME="{home}"
afc_path="{afc_path}"
moonraker_config_file="$(mktemp)"
update_moonraker_config
cat "$moonraker_config_file"
rm -f "$moonraker_config_file"
"""
    proc = subprocess.run(
        ["bash", "-c", script], capture_output=True, text=True, timeout=30
    )
    assert proc.returncode == 0, proc.stderr
    return proc.stdout


def test_moonraker_update_manager_uses_home_relative_path():
    out = _run_moonraker_update("/root/AFC-Klipper-Add-On", "/root")
    assert "path: ~/AFC-Klipper-Add-On" in out, out


def test_moonraker_update_manager_uses_absolute_path_outside_home():
    # e.g. a K1 clone under /usr/data
    out = _run_moonraker_update("/usr/data/AFC-Klipper-Add-On", "/root")
    assert "path: /usr/data/AFC-Klipper-Add-On" in out, out
    assert "path: ~/AFC-Klipper-Add-On" not in out, out
