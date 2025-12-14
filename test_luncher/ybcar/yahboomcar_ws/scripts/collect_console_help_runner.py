#!/usr/bin/env python3
"""Locate `console_scripts` from package `setup.py` files and collect their `--help` output.

Usage:
  python3 scripts/collect_console_help_runner.py [--run] [--out-dir PATH] [--timeout N] [--force]

By default this is a dry-run that prints the commands it would run. Use `--run` to execute.
The script refuses to execute unless a ROS environment is likely present (env var `ROS_DISTRO`) or `--force` is passed.
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path
import shutil
import os


def find_console_scripts(base: Path):
    found = []
    for setup in base.rglob('setup.py'):
        try:
            txt = setup.read_text(encoding='utf-8')
        except Exception:
            continue
        m = re.search(r"console_scripts\s*=\s*\[(.*?)\]", txt, re.S)
        if not m:
            continue
        block = m.group(1)
        entries = [s for s in re.findall(r"['\"]([^'\"]+?)['\"]", block) if s.strip()]
        pkg = setup.parent.name
        for entry in entries:
            # entry format: "name = module:callable"
            name = entry.split('=')[0].strip()
            found.append((pkg, name))
    return found


def run_help(cmd, timeout):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout + '\n' + p.stderr
    except Exception as e:
        return 255, f'ERROR: {e}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='store_true', help='Actually execute commands')
    parser.add_argument('--out-dir', default='scripts/ROS2_HELP', help='Directory to save outputs')
    parser.add_argument('--timeout', type=int, default=10, help='Per-command timeout (s)')
    parser.add_argument('--force', action='store_true', help='Force execution even if ROS env not detected')
    args = parser.parse_args()

    base = Path(__file__).resolve().parents[3] / 'test_luncher' / 'ybcar'
    console = find_console_scripts(base)

    if not console:
        print('No console_scripts found in setup.py files under', base)
        return

    commands = []
    for pkg, exe in console:
        cmd = ['ros2', 'run', pkg, exe, '--ros-args', '--help']
        commands.append((pkg, exe, cmd))

    if not args.run:
        print('Dry-run: the following commands would be executed:')
        for pkg, exe, cmd in commands:
            print(' ', ' '.join(cmd))
        print('\nTo execute, re-run with --run (requires ROS2 environment or --force).')
        return

    # safety checks
    if not args.force and 'ROS_DISTRO' not in os.environ:
        print('ROS_DISTRO not found in environment. Source your ROS2 setup or pass --force to override.')
        return

    if shutil.which('ros2') is None:
        print('`ros2` command not found on PATH. Ensure ROS2 is installed and sourced.')
        return

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for pkg, exe, cmd in commands:
        fname = f'{pkg}__{exe}.txt'
        out_path = out_dir / fname
        print('Running:', ' '.join(cmd))
        code, text = run_help(cmd, timeout=args.timeout)
        out_path.write_text(text, encoding='utf-8')
        print('Wrote', out_path, 'returncode', code)


if __name__ == '__main__':
    main()
