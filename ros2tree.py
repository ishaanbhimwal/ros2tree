#!/usr/bin/env python3
import argparse
import subprocess
import shutil
import sys
import os


def main():
    parser = argparse.ArgumentParser(prog="ros2tree", add_help=False)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("-n", "--node", action="store_true", help="view nodes")
    group.add_argument("-t", "--topic", action="store_true", help="view topics")
    opts, forwarded = parser.parse_known_args()

    target = "ros2tree_node" if opts.node else "ros2tree_topic"
    exe = shutil.which(target)

    if not exe:
        here = os.path.dirname(os.path.abspath(__file__))
        candidate = os.path.join(here, target)
        if os.path.exists(candidate):
            exe = candidate
        else:
            print(f"Error: cannot find '{target}' on PATH.", file=sys.stderr)
            sys.exit(127)

    sys.exit(subprocess.call([exe, *forwarded]))


if __name__ == "__main__":
    main()
