#!/usr/bin/env python3
"""Statically extract argparse `add_argument` options from python modules.

Usage:
  python3 scripts/extract_cli_options.py > scripts/CLI_OPTIONS.md

This script walks `test_luncher/ybcar/**/src` for .py files, parses AST,
and finds calls to `add_argument` to list CLI flags and their help/defaults.
"""
import ast
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
# By default we scan package `src` directories. Expand to include `scripts` and `install` folders
SEARCH_DIRS = ["src", "scripts", "install"]

def parse_file(path: Path):
    try:
        src = path.read_text(encoding='utf-8')
    except Exception:
        return []
    try:
        tree = ast.parse(src)
    except SyntaxError:
        return []
    entries = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'attr', '') == 'add_argument':
            # collect positional/string flags
            flags = []
            help_text = None
            default = None
            for a in node.args:
                if isinstance(a, ast.Constant) and isinstance(a.value, str):
                    flags.append(a.value)
                elif isinstance(a, (ast.List, ast.Tuple)):
                    vals = [elt.s for elt in a.elts if isinstance(elt, ast.Constant)]
                    flags.extend(vals)
            for kw in node.keywords:
                if kw.arg == 'help' and isinstance(kw.value, ast.Constant):
                    help_text = kw.value.value
                if kw.arg == 'default' and isinstance(kw.value, ast.Constant):
                    default = kw.value.value
            entries.append({'flags': flags, 'help': help_text, 'default': default})
        # detect click.option / click.argument used as decorators
        if isinstance(node, ast.FunctionDef):
            for dec in node.decorator_list:
                if isinstance(dec, ast.Call) and isinstance(dec.func, ast.Attribute):
                    if getattr(dec.func.value, 'id', '') == 'click' and dec.func.attr in ('option', 'argument'):
                        flags = []
                        help_text = None
                        default = None
                        # decorator args often contain the flag string(s)
                        for a in dec.args:
                            if isinstance(a, ast.Constant) and isinstance(a.value, str):
                                flags.append(a.value)
                            elif isinstance(a, (ast.List, ast.Tuple)):
                                vals = [elt.s for elt in a.elts if isinstance(elt, ast.Constant)]
                                flags.extend(vals)
                        for kw in dec.keywords:
                            if kw.arg == 'help' and isinstance(kw.value, ast.Constant):
                                help_text = kw.value.value
                            if kw.arg == 'default' and isinstance(kw.value, ast.Constant):
                                default = kw.value.value
                        entries.append({'flags': flags or [f'<{node.name}>'], 'help': help_text, 'default': default})
    return entries

def main():
    results = {}
    # Walk packages under test_luncher/ybcar and scan selected directories
    base = Path(__file__).resolve().parents[3] / 'test_luncher' / 'ybcar'
    for dname in SEARCH_DIRS:
        for p in base.rglob(dname):
            if not p.is_dir():
                continue
            for py in p.rglob('*.py'):
                try:
                    rel = py.relative_to(base)
                except Exception:
                    rel = py
                opts = parse_file(py)
                if opts:
                    results[str(rel)] = opts
    # Additionally scan for setup.py files and extract console_scripts entry_points
    for setup in base.rglob('setup.py'):
        try:
            txt = setup.read_text(encoding='utf-8')
        except Exception:
            continue
        # crude parse: look for console_scripts block
        import re
        m = re.search(r"console_scripts\s*=\s*\[(.*?)\]", txt, re.S)
        if m:
            entries = []
            block = m.group(1)
            for line in re.findall(r"['\"]([^'\"]+?)['\"]", block):
                # each line like 'name = module:callable'
                entries.append({'flags': [line.split('=')[0].strip()], 'help': f'console_script -> {line}', 'default': None})
            if entries:
                results[str(setup.relative_to(base)) + ' (console_scripts)'] = entries

    # Emit markdown
    print('# CLI Options (auto-extracted)')
    print()
    if not results:
        print('No `add_argument` calls found by static parse.')
        return
    for fname, opts in sorted(results.items()):
        print('##', fname)
        for o in opts:
            flags = ', '.join(o['flags']) if o['flags'] else '(positional)'
            h = o['help'] or ''
            d = f" (default={o['default']})" if o['default'] is not None else ''
            print(f'- **{flags}**: {h}{d}')
        print()

if __name__ == '__main__':
    main()
