#!/usr/bin/env python3
"""Preserve failed fixture/report attempts and point the suite at the latest."""
import argparse
import os
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    if args.output.exists() and not args.output.is_symlink():
        parser.error('Output alias must be new or a managed symlink')
    index = 1
    while args.output.with_name(args.output.name+f'_attempt{index}').exists():
        index += 1
    actual = args.output.with_name(args.output.name+f'_attempt{index}')
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('A command is required')
    command = [arg.replace('{OUTPUT}', str(actual)) for arg in command]
    process = subprocess.run(command)
    if actual.exists():
        temporary = args.output.with_name(args.output.name+'.partial')
        temporary.symlink_to(actual.resolve())
        os.replace(temporary, args.output)
    raise SystemExit(process.returncode)


if __name__ == '__main__':
    main()
