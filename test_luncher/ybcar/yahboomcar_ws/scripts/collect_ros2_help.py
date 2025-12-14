#!/usr/bin/env python3
"""Collect `ros2 run <pkg> <exe> --ros-args --help` outputs for console_scripts.

Usage:
  python3 scripts/collect_ros2_help.py [--run]

Without `--run` this script prints discovered console scripts. With `--run` it
will attempt to invoke `ros2 run <pkg> <exe> --ros-args --help` and save outputs
under `scripts/ROS2_HELP/`.

Note: must be executed in a sourced ROS2 environment where `ros2` is available.
"""
import ast
import os
import subprocess
from pathlib import Path
import argparse

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'src'

def find_entry_points():
    results = []
    for setup in ROOT.parents[0].rglob('setup.py'):
        try:
            text = setup.read_text(encoding='utf-8')
        except Exception:
            continue
        if 'console_scripts' in text:
            # crude parsing: find lines within entry_points block
            lines = text.splitlines()
            pkg = setup.parent.name
            for i,l in enumerate(lines):
                if 'console_scripts' in l:
                    # capture next 20 lines for entries
                    for ent in lines[i:i+40]:
                        if '=' in ent and '(' not in ent:
                            # format: 'name= module:call'
                            part = ent.strip().strip("',[]")
                            if '=' in part:
                                name = part.split('=')[0].strip()
                                results.append((pkg, name))
    return results

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='store_true')
    args = parser.parse_args()
    eps = find_entry_points()
    print('Discovered console scripts:')
    for pkg,exe in eps:
        print(f'- {pkg} :: {exe}')
    if args.run:
        outdir = ROOT / 'ROS2_HELP'
        outdir.mkdir(exist_ok=True)
        for pkg,exe in eps:
            fname = f"{pkg}__{exe}.txt"
            path = outdir / fname
            cmd = ['ros2','run',pkg,exe,'--ros-args','--help']
            try:
                print('Running', ' '.join(cmd))
                out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
            except subprocess.CalledProcessError as e:
                out = e.output
            except FileNotFoundError:
                out = 'ros2 not found in PATH or not sourced ROS 2 environment.'
            path.write_text(out, encoding='utf-8')
            print('Wrote', path)

if __name__ == '__main__':
    main()
