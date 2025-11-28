# topics.py

import argparse
import os
import rclpy
from rclpy.node import Node
import time
from rosidl_runtime_py.utilities import get_message


# Utils


def fq(ns, name):
    return f"/{name}" if ns == "/" else f"{ns}/{name}"


def qos_str(qos):
    try:
        rel = qos.reliability.name.lower()
        dur = qos.durability.name.lower()
        depth = qos.depth
    except Exception:
        rel = str(qos.reliability)
        dur = str(qos.durability)
        depth = getattr(qos, "depth", "?")
    return f"{rel}, {dur}, depth={depth}"


class Inspector(Node):
    def __init__(self):
        super().__init__("topic_tree_inspector")


def is_hidden_topic(name: str) -> bool:
    parts = [p for p in name.split("/") if p]
    if any(part.startswith("_") for part in parts):
        return True
    hidden_basenames = {"parameter_events", "rosout"}
    return parts and parts[-1] in hidden_basenames


def compute_topic_node_counts(tree) -> tuple[int, int]:
    node_set = set()
    topic_count = 0

    def walk(subtree):
        nonlocal topic_count
        if not isinstance(subtree, dict):
            return
        for v in subtree.values():
            if isinstance(v, dict) and "__type__" in v:
                topic_count += 1
                for node_name, _qos in v.get("pubs", []):
                    node_set.add(node_name)
                for node_name, _qos in v.get("subs", []):
                    node_set.add(node_name)
            elif isinstance(v, dict):
                walk(v)

    walk(tree)
    return len(node_set), topic_count


def build_tree(node, include_hidden=False):
    tree = {}
    for topic, types in node.get_topic_names_and_types():
        if not include_hidden and is_hidden_topic(topic):
            continue

        pubs = node.get_publishers_info_by_topic(topic)
        subs = node.get_subscriptions_info_by_topic(topic)

        parts = [p for p in topic.split("/") if p]
        cur = tree
        for p in parts[:-1]:
            cur = cur.setdefault(p, {})
        leaf = cur.setdefault(
            parts[-1], {"__type__": ",".join(types), "pubs": [], "subs": []}
        )

        for info in pubs:
            leaf["pubs"].append(
                (fq(info.node_namespace, info.node_name), qos_str(info.qos_profile))
            )
        for info in subs:
            leaf["subs"].append(
                (fq(info.node_namespace, info.node_name), qos_str(info.qos_profile))
            )
    return tree


# Colors

DEFAULT_LS = {"di": "01;34", "ln": "01;36", "ex": "01;35", "fi": "0"}


def parse_ls_colors():
    # Prefer GNU LS_COLORS if present, return di/ln/ex/fi with sensible defaults
    ls = os.environ.get("LS_COLORS", "")
    if ls:
        colors = {}
        for part in ls.split(":"):
            if "=" in part:
                k, v = part.split("=", 1)
                colors[k] = v
        return {k: colors.get(k, v) for k, v in DEFAULT_LS.items()}
    # If LSCOLORS is present (BSD/macOS), use sensible defaults
    if os.environ.get("LSCOLORS"):
        return dict(DEFAULT_LS)
    # Fallback defaults
    return dict(DEFAULT_LS)


def colorize(text, code):
    return f"\x1b[{code}m{text}\x1b[0m" if code else text


# Printing


def print_tree(
    tree,
    prefix="",
    enable_color=False,
    ls_colors=None,
    show_type=False,
    show_qos=False,
    show_hz=False,
):
    # Colors from LS_COLORS/LSCOLORS
    di = (ls_colors or {}).get("di", "01;34")  # Topic names
    ln = (ls_colors or {}).get("ln", "01;36")  # Publisher color
    ex = (ls_colors or {}).get("ex", "01;35")  # Subscription color

    keys = sorted(tree.keys())
    for i, k in enumerate(keys):
        last = i == len(keys) - 1
        branch = "└─ " if last else "├─ "
        next_prefix = prefix + ("   " if last else "│  ")
        val = tree[k]

        if isinstance(val, dict) and "__type__" in val:
            # Topic leaf name
            name = colorize(k, di) if enable_color else k
            type_str = f' [{val["__type__"]}]' if show_type else ""
            hz = val.get("__hz__")
            hz_str = (
                f" [{hz:.1f} Hz]"
                if (show_hz and hz is not None)
                else (" [no data]" if show_hz else "")
            )
            print(f"{prefix}{branch}{name}{type_str}{hz_str}")

            # Publishers
            for j, (node_name, qos) in enumerate(sorted(val["pubs"])):
                br = "├─ " if (j + 1) < len(val["pubs"]) or val["subs"] else "└─ "
                label = colorize("pub:", ln) if enable_color else "pub:"
                node_disp = colorize(node_name, ln) if enable_color else node_name
                qos_part = f" ({qos})" if show_qos else ""
                print(f"{next_prefix}{br}{label} {node_disp}{qos_part}")

            # Subscriptions
            for j, (node_name, qos) in enumerate(sorted(val["subs"])):
                br = "└─ " if (j + 1) == len(val["subs"]) else "├─ "
                label = colorize("sub:", ex) if enable_color else "sub:"
                node_disp = colorize(node_name, ex) if enable_color else node_name
                qos_part = f" ({qos})" if show_qos else ""
                print(f"{next_prefix}{br}{label} {node_disp}{qos_part}")

        else:
            # Directory segments
            dir_name = k + "/"
            dir_name = colorize(dir_name, di) if enable_color else dir_name
            print(f"{prefix}{branch}{dir_name}")
            print_tree(
                val, next_prefix, enable_color, ls_colors, show_type, show_qos, show_hz
            )


# Hz Measurement


def measure_and_annotate_hz(node, tree, include_hidden=False, window_sec=1.0):
    # Map fully-qualified topic -> leaf dict in the tree
    topic_to_leaf = {}

    def leaf_for_topic(topic):
        parts = [p for p in topic.split("/") if p]
        cur = tree
        for p in parts[:-1]:
            cur = cur.get(p)
            if cur is None:
                return None
        leaf = cur.get(parts[-1]) if cur else None
        return leaf if isinstance(leaf, dict) and "__type__" in leaf else None

    # Prepare subscriptions and counters
    subs = []
    counts = {}
    t_first = {}
    t_last = {}

    def make_cb(topic):
        def _cb(msg):
            now = time.monotonic()
            counts[topic] = counts.get(topic, 0) + 1
            if topic not in t_first:
                t_first[topic] = now
            t_last[topic] = now

        return _cb

    for topic, types in node.get_topic_names_and_types():
        if not include_hidden and is_hidden_topic(topic):
            continue
        if not types:
            continue

        leaf = leaf_for_topic(topic)
        if not leaf:
            continue

        try:
            msg_type = get_message(types[0])
        except Exception:
            continue

        subs.append(node.create_subscription(msg_type, topic, make_cb(topic), 10))
        topic_to_leaf[topic] = leaf
        leaf["__hz__"] = None

    end = time.monotonic() + max(window_sec, 0.1)
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)

    for topic, leaf in topic_to_leaf.items():
        n = counts.get(topic, 0)
        if n == 0:
            leaf["__hz__"] = None
        elif n == 1:
            duration = max(
                time.monotonic() - t_first.get(topic, end - window_sec), 1e-3
            )
            leaf["__hz__"] = 1.0 / duration
        else:
            duration = max(t_last[topic] - t_first[topic], 1e-3)
            leaf["__hz__"] = n / duration


# Filtering


def filter_tree_to_path(tree, path):
    parts = [p for p in path.split("/") if p]
    if not parts:
        return tree
    cur = tree
    for p in parts:
        if not isinstance(cur, dict) or p not in cur:
            return {}
        cur = cur[p]
    sub = cur
    for p in reversed(parts):
        sub = {p: sub}
    return sub


# CLI


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        formatter_class=lambda prog: argparse.HelpFormatter(
            prog, max_help_position=40, width=120
        )
    )
    parser.add_argument(
        "-c", "--show-color", action="store_true", help="colorize output"
    )
    parser.add_argument(
        "-m",
        "--show-msg-type",
        action="store_true",
        help="show message type next to topic",
    )
    parser.add_argument(
        "-q", "--show-qos", action="store_true", help="show QoS after pub/sub lines"
    )
    parser.add_argument(
        "-T",
        "--show-topic",
        default=None,
        help="only show this topic or namespace path (e.g., /fmu/out/vehicle_status or /fmu)",
    )
    parser.add_argument(
        "-i",
        "--include-hidden-topics",
        action="store_true",
        help='show topics with any path segment starting with "_" (e.g., _action)',
    )
    parser.add_argument(
        "-H",
        "--show-hz",
        action="store_true",
        help="sample and show message rate (Hz) next to each topic",
    )
    parser.add_argument(
        "--hz-window",
        type=float,
        default=1.0,
        help="sampling window in seconds for Hz measurement (default: 1.0)",
    )
    return parser.parse_args(argv)


# Main


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = Inspector()
    try:
        time.sleep(0.5)  # Allow discovery to settle
        tree = build_tree(node, include_hidden=args.include_hidden_topics)

        if args.show_topic:
            sel = (
                args.show_topic
                if args.show_topic.startswith("/")
                else "/" + args.show_topic
            )
            tree = filter_tree_to_path(tree, sel)
            if not tree:
                print(f"[warn] not found: {sel}")
                return

        if args.show_hz:
            measure_and_annotate_hz(
                node,
                tree,
                include_hidden=args.include_hidden_topics,
                window_sec=args.hz_window,
            )

        ls_colors = parse_ls_colors() if args.show_color else None
        ns = node.get_namespace() or "/"
        ns = ns if ns.startswith("/") else "/" + ns
        print(ns if ns != "" else "/")

        print_tree(
            tree,
            enable_color=args.show_color,
            ls_colors=ls_colors,
            show_type=args.show_msg_type,
            show_qos=args.show_qos,
            show_hz=args.show_hz,
        )

        n_nodes, n_topics = compute_topic_node_counts(tree)
        print(f"\n{n_topics} topics, {n_nodes} nodes")
    finally:
        rclpy.shutdown()
