#!/usr/bin/env python3

# ros2tree_node.py

import argparse
import os
import re
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import time
import rclpy
from rclpy.node import Node

try:
    from ament_index_python.packages import get_packages_with_prefixes
except Exception:
    get_packages_with_prefixes = None

# ---- utils / shared helpers ----


def fq(ns: str, name: str) -> str:
    if not ns or ns == "/":
        return f"/{name}"
    return ns.rstrip("/") + "/" + name


def base_name_from_fqn(fqn: str) -> str:
    return fqn.rsplit("/", 1)[-1] if fqn else fqn


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


def is_hidden_topic(name: str) -> bool:
    parts = [p for p in name.split("/") if p]
    if any(part.startswith("_") for part in parts):
        return True
    hidden_basenames = {"parameter_events", "rosout"}
    return parts and parts[-1] in hidden_basenames


def compute_node_topic_counts(
    node_graph: Dict[str, dict],
    node_filter: Optional[str] = None,
) -> Tuple[int, int]:
    # Apply same filter logic as print_nodes_tree
    def matches_filter(fqn: str) -> bool:
        if not node_filter:
            return True
        sel = node_filter if node_filter.startswith("/") else "/" + node_filter
        return fqn == sel or fqn.startswith(sel + "/")

    nodes = [f for f in node_graph.keys() if matches_filter(f)]
    topic_set = set()
    for fqn in nodes:
        for topic, *_ in node_graph.get(fqn, {}).get("pubs", []):
            topic_set.add(topic)
        for topic, *_ in node_graph.get(fqn, {}).get("subs", []):
            topic_set.add(topic)
    return len(nodes), len(topic_set)


DEFAULT_LS = {"di": "01;34", "ln": "01;36", "ex": "01;35", "fi": "0"}


def parse_ls_colors():
    ls = os.environ.get("LS_COLORS", "")
    if ls:
        colors = {}
        for part in ls.split(":"):
            if "=" in part:
                k, v = part.split("=", 1)
                colors[k] = v
        return {k: colors.get(k, v) for k, v in DEFAULT_LS.items()}
    if os.environ.get("LSCOLORS"):
        return dict(DEFAULT_LS)
    return dict(DEFAULT_LS)


def colorize(text, code, enable=True):
    return f"\x1b[{code}m{text}\x1b[0m" if (enable and code) else text


# ---- rclpy compatibility ----


def safe_get_topic_names_and_types(n: Node) -> List[Tuple[str, List[str]]]:
    try:
        # No include_hidden_topics for node view
        return n.get_topic_names_and_types()
    except Exception:
        return []


def safe_get_node_names_and_namespaces(n: Node) -> List[Tuple[str, str]]:
    try:
        return n.get_node_names_and_namespaces()
    except Exception:
        try:
            names = n.get_node_names()
            return [(name, "/") for name in names]
        except Exception:
            return []


# ---- internal visibility toggle ----
# hide internal nodes (ros2cli daemon and introspector).
# flip this based on -i/--include-hidden-topics to match topic-view behavior.
HIDE_INTERNAL = True


# ---- hide helpers ----


def should_hide_node(fqn: str) -> bool:
    """Hide ros2cli internals and our introspector."""
    """Hide ros2cli internals and our introspector (unless HIDE_INTERNAL is False)."""

    if not HIDE_INTERNAL:
        return False

    base = base_name_from_fqn(fqn) or ""
    return (
        fqn == "/ros2_tree_introspector"
        or fqn.startswith("/_ros2cli")
        or base.startswith("_ros2cli")
        or base.startswith("ros2cli")
    )


# ---- ROS graph (augmented with types/QoS) ----


def build_node_graph(
    n: Node, include_hidden: bool = False
) -> Dict[str, Dict[str, List[Tuple[str, str, str]]]]:
    """
    Returns:
      {
        '/ns/node': {
           'pubs': [(topic, type, qos), ...],
           'subs': [(topic, type, qos), ...],
        },
        ...
      }
    """
    node_graph: Dict[str, Dict[str, List[Tuple[str, str, str]]]] = {}
    for name, ns in safe_get_node_names_and_namespaces(n):
        fqn_node = fq(ns, name)
        if should_hide_node(fqn_node):
            continue
        node_graph.setdefault(fqn_node, {"pubs": [], "subs": []})

    t_list = safe_get_topic_names_and_types(n)
    if not include_hidden:
        t_list = [(t, types) for t, types in t_list if not is_hidden_topic(t)]

    topic_types = {t: ",".join(types) if types else "?" for t, types in t_list}

    for topic, _types in t_list:
        # pubs
        try:
            pubs = n.get_publishers_info_by_topic(topic)
        except Exception:
            pubs = []
        for info in pubs:
            fqn = fq(info.node_namespace, info.node_name)
            if should_hide_node(fqn):
                continue
            node_graph.setdefault(fqn, {"pubs": [], "subs": []})
            node_graph[fqn]["pubs"].append(
                (topic, topic_types.get(topic, "?"), qos_str(info.qos_profile))
            )
        # subs
        try:
            subs = n.get_subscriptions_info_by_topic(topic)
        except Exception:
            subs = []
        for info in subs:
            fqn = fq(info.node_namespace, info.node_name)
            if should_hide_node(fqn):
                continue
            node_graph.setdefault(fqn, {"pubs": [], "subs": []})
            node_graph[fqn]["subs"].append(
                (topic, topic_types.get(topic, "?"), qos_str(info.qos_profile))
            )
    return node_graph


# ---- ament executable index ----

_EXEC_INDEX: Dict[str, List[Tuple[str, str]]] = {}


def build_exec_index() -> None:
    global _EXEC_INDEX
    _EXEC_INDEX = {}
    if not get_packages_with_prefixes:
        return
    try:
        packages = get_packages_with_prefixes()
    except Exception:
        return
    for pkg, prefix in packages.items():
        lib_dir = Path(prefix) / "lib" / pkg
        if not lib_dir.exists():
            continue
        try:
            for entry in lib_dir.iterdir():
                if entry.is_file() and os.access(entry, os.X_OK):
                    _EXEC_INDEX.setdefault(entry.name, []).append(
                        (pkg, str(entry.resolve()))
                    )
        except Exception:
            continue


def resolve_exec_by_index(name: str) -> Optional[Tuple[str, str]]:
    if not name:
        return None
    cands = _EXEC_INDEX.get(name, [])
    return cands[0] if len(cands) == 1 else None


# ---- /proc helpers (Linux) ----

CMD_NULL = "\x00"
NODE_RE = re.compile(r"__node:=([^\s\0]+)")
NS_RE = re.compile(r"__ns:=([^\s\0]+)")
PY_RUNNERS = {
    "python",
    "python3",
    "python3.8",
    "python3.9",
    "python3.10",
    "python3.11",
    "python3.12",
}


def read_cmdline(pid: int) -> List[str]:
    try:
        data = Path(f"/proc/{pid}/cmdline").read_bytes()
        if not data:
            return []
        parts = data.decode(errors="ignore").split(CMD_NULL)
        return [p for p in parts if p]
    except Exception:
        return []


def read_exe(pid: int) -> Optional[str]:
    try:
        return os.readlink(f"/proc/{pid}/exe")
    except Exception:
        return None


def guess_pkg_exec_from_path(path: str) -> Tuple[Optional[str], Optional[str], bool]:
    """Returns (pkg, exe, is_installed_lib_path)"""
    p = Path(path)
    try:
        parts = p.resolve().parts
    except Exception:
        parts = p.parts

    # .../lib/<pkg>/<exec> - INSTALLED BINARY (C++ or Python)
    try:
        lib_idx = parts.index("lib")
        if lib_idx + 2 < len(parts):
            pkg = parts[lib_idx + 1]
            exe = parts[lib_idx + 2]
            return pkg, exe, True
    except ValueError:
        pass

    # .../src/<pkg>/... - dev source
    try:
        src_idx = parts.index("src")
        if src_idx + 1 < len(parts):
            return parts[src_idx + 1], p.stem, False
    except ValueError:
        pass

    return None, p.name, False


def resolve_script_or_exe(
    argv0: str, cmd: List[str], exe_link: Optional[str]
) -> Tuple[Optional[str], Optional[str], Optional[str], bool]:
    """Returns (pkg, exe, full_path, is_confident)"""

    # PRIORITY 1: /proc/pid/exe link (works for C++ binaries)
    if exe_link:
        pkg, exe, confident = guess_pkg_exec_from_path(exe_link)
        if pkg and confident:
            return pkg, exe, exe_link, True

    # PRIORITY 2: Python runner with script arg
    if Path(argv0).name in PY_RUNNERS and len(cmd) >= 2:
        arg1 = cmd[1]
        if os.path.isabs(arg1) and os.path.exists(arg1):
            pkg, exe, confident = guess_pkg_exec_from_path(arg1)
            return pkg, exe, arg1, confident
        # Try index
        res = resolve_exec_by_index(Path(arg1).name)
        if res:
            pkg, full = res
            return pkg, Path(full).name, full, True

    # PRIORITY 3: Absolute argv0
    if os.path.isabs(argv0) and os.path.exists(argv0):
        pkg, exe, confident = guess_pkg_exec_from_path(argv0)
        return pkg, exe, argv0, confident

    # PRIORITY 4: Index lookup by name
    res = resolve_exec_by_index(Path(argv0).name)
    if res:
        pkg, full = res
        return pkg, Path(full).name, full, True

    # PRIORITY 5: Fallback to exe_link even if not confident
    if exe_link:
        pkg, exe, confident = guess_pkg_exec_from_path(exe_link)
        return pkg, exe, exe_link, confident

    return None, None, None, False


def find_processes(hide_cli_daemon: bool = True) -> List[dict]:
    if not _EXEC_INDEX:
        build_exec_index()
    self_pid = os.getpid()
    procs = []
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid == self_pid:
            continue
        cmd = read_cmdline(pid)
        if not cmd:
            continue
        exe_link = read_exe(pid)

        argv0 = cmd[0]
        pkg, exe, path_candidate, is_confident = resolve_script_or_exe(
            argv0, cmd, exe_link
        )

        raw = CMD_NULL.join(cmd)
        m_node = NODE_RE.search(raw)
        m_ns = NS_RE.search(raw)
        node_name = m_node.group(1) if m_node else None
        node_ns = m_ns.group(1) if m_ns else None

        cmd_str = " ".join(cmd)
        is_wrapper = (
            "/bin/ros2" in argv0 or "/bin/ros2" in (path_candidate or "")
        ) and " run " in cmd_str
        is_cli_daemon = "_ros2cli_daemon_" in cmd_str or (exe and "ros2cli" in exe)
        is_pkg_exe = bool(path_candidate) and "/lib/" in path_candidate

        if hide_cli_daemon and is_cli_daemon:
            continue

        procs.append(
            {
                "pid": pid,
                "argv0": argv0,
                "path": path_candidate,
                "is_confident": is_confident,
                "pkg": pkg,
                "exe": exe,
                "node_name": node_name,
                "node_ns": node_ns or None,
                "is_wrapper": is_wrapper,
                "is_pkg_exe": is_pkg_exe,
                "cmd_str": cmd_str,
            }
        )
    return procs


# ---- mapping heuristics ----


def names_match(node_base: str, exe_base: str) -> bool:
    if node_base == exe_base:
        return True
    # plural/singular
    if exe_base.endswith("s") and node_base == exe_base[:-1]:
        return True
    if node_base.endswith("s") and exe_base == node_base[:-1]:
        return True
    # underscores
    if node_base.replace("_", "") == exe_base.replace("_", ""):
        return True
    return False


def map_nodes_to_processes(
    node_graph: Dict[str, dict], procs: List[dict]
) -> Dict[str, dict]:
    mapping: Dict[str, dict] = {}
    if not node_graph:
        return mapping

    user_nodes = [f for f in node_graph.keys() if not should_hide_node(f)]
    if not user_nodes:
        return mapping

    # Pass 1: explicit __node/__ns
    for p in procs:
        if p.get("node_name"):
            ns = p.get("node_ns") or "/"
            fqn = fq(ns, p["node_name"])
            if should_hide_node(fqn):
                continue
            if fqn in node_graph and fqn not in mapping:
                q = dict(p)
                q["mapping_method"] = "explicit"
                mapping[fqn] = q

    if len(mapping) == len(user_nodes):
        return mapping

    # Consider only installed executables (avoid ros2 wrappers)
    pkg_exe_procs = [
        p for p in procs if p.get("is_pkg_exe") and not p.get("is_wrapper")
    ]

    # Pass 2: singleton installed exe
    if (
        len(user_nodes) == 1
        and len(pkg_exe_procs) == 1
        and user_nodes[0] not in mapping
    ):
        q = dict(pkg_exe_procs[0])
        q["mapping_method"] = "pkg_exe_single"
        mapping[user_nodes[0]] = q
        return mapping

    # Pass 3: name match
    by_exe: Dict[str, List[dict]] = {}
    for p in pkg_exe_procs:
        exe_base = Path(p.get("exe") or "").name
        if exe_base:
            by_exe.setdefault(exe_base, []).append(p)

    for fqn in user_nodes:
        if fqn in mapping:
            continue
        node_base = base_name_from_fqn(fqn)
        candidates = []
        for exe_name, plist in by_exe.items():
            if names_match(node_base, exe_name):
                candidates.extend(plist)
        if len(candidates) == 1:
            q = dict(candidates[0])
            q["mapping_method"] = "name_match"
            mapping[fqn] = q

    if len(mapping) == len(user_nodes):
        return mapping

    # Pass 4: singleton fallback
    unmapped = [f for f in user_nodes if f not in mapping]
    remaining_pkg_exe = [p for p in pkg_exe_procs if p not in mapping.values()]
    if len(unmapped) == 1 and len(remaining_pkg_exe) == 1:
        q = dict(remaining_pkg_exe[0])
        q["mapping_method"] = "singleton_fallback"
        mapping[unmapped[0]] = q

    return mapping


# ---- printing ----


def print_nodes_tree(
    node_graph: Dict[str, dict],
    mapping: Dict[str, dict],
    *,
    enable_color=False,
    ls_colors=None,
    show_type=False,
    show_qos=False,
    node_filter: Optional[str] = None,
    show_pid=False,
    show_pkg=False,
):
    di = (ls_colors or {}).get("di", "01;34")  # node names
    ln = (ls_colors or {}).get("ln", "01;36")  # pubs color
    ex = (ls_colors or {}).get("ex", "01;35")  # subs color

    def matches_filter(fqn: str) -> bool:
        if not node_filter:
            return True
        sel = node_filter if node_filter.startswith("/") else "/" + node_filter
        return fqn == sel or fqn.startswith(sel + "/")

    nodes = [f for f in node_graph.keys() if matches_filter(f)]
    nodes.sort()

    for idx, fqn_node in enumerate(nodes):
        is_last_node = idx == len(nodes) - 1
        branch = "└─ " if is_last_node else "├─ "
        next_prefix = "   " if is_last_node else "│  "
        node_disp = colorize(fqn_node, di, enable_color)
        proc = mapping.get(fqn_node, {})

        header = f"{branch}{node_disp}"
        attrs = []
        if show_pid:
            pid_str = str(proc.get("pid")) if proc and proc.get("pid") else "?"
            attrs.append(f"[PID:{pid_str}]")
        if show_pkg:
            pkg = proc.get("pkg") or "?"
            attrs.append(f"[pkg:{pkg}]")
        if attrs:
            header += " " + " ".join(attrs)
        print(header)

        pubs = sorted(set(node_graph[fqn_node].get("pubs", [])))
        subs = sorted(set(node_graph[fqn_node].get("subs", [])))

        pubs_label = colorize("pubs:", ln, enable_color)
        subs_label = colorize("subs:", ex, enable_color)

        has_pubs = bool(pubs)
        has_subs = bool(subs)

        # Print pubs section ONLY if there are pubs
        if has_pubs:
            # Label branch depends on whether subs section follows
            label_br = "├─ " if has_subs else "└─ "
            print(f"{next_prefix}{label_br}{pubs_label}")
            # Items: keep vertical line if subs follows
            mid = "│  " if has_subs else "   "
            for j, item in enumerate(pubs):
                # If subs follows, keep '├─ ' even for last pub item to visually continue
                br = "└─ " if (j == len(pubs) - 1 and not has_subs) else "├─ "
                topic, typ, qos = item if len(item) == 3 else (item, "?", "")
                type_part = f" [{typ}]" if show_type else ""
                qos_part = f" ({qos})" if (show_qos and qos) else ""
                prefix = f"{next_prefix}{mid}{br}"
                topic_col = colorize(topic, ln, enable_color)
                print(prefix + topic_col + type_part + qos_part)

        # Print subs section (if any)
        if has_subs:
            print(f"{next_prefix}└─ {subs_label}")
            for j, item in enumerate(subs):
                br = "└─ " if j == len(subs) - 1 else "├─ "
                topic, typ, qos = item if len(item) == 3 else (item, "?", "")
                type_part = f" [{typ}]" if show_type else ""
                qos_part = f" ({qos})" if (show_qos and qos) else ""
                prefix = f"{next_prefix}   {br}"
                topic_col = colorize(topic, ex, enable_color)
                print(prefix + topic_col + type_part + qos_part)
        elif not has_pubs:
            # show that both sections are empty
            # print(f"{next_prefix}└─ {subs_label} (none)")
            pass


# ---- main ----


class Introspector(Node):
    def __init__(self):
        super().__init__("ros2_tree_introspector")


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        formatter_class=lambda prog: argparse.HelpFormatter(
            prog, max_help_position=40, width=120
        )
    )
    p.add_argument("-c", "--show-color", action="store_true", help="colorize output")
    p.add_argument(
        "-m",
        "--show-msg-type",
        action="store_true",
        help="show message type next to topic",
    )
    p.add_argument(
        "-q", "--show-qos", action="store_true", help="show QoS after pub/sub lines"
    )
    p.add_argument(
        "-i",
        "--include-hidden-topics",
        action="store_true",
        help='show topics with any path segment starting with "_" (e.g., _action)',
    )
    p.add_argument(
        "-N",
        "--show-node",
        default=None,
        help="filter by node or namespace (e.g., /ns/node or /ns)",
    )
    p.add_argument(
        "-P", "--show-pid", action="store_true", help="show PID for each node"
    )
    p.add_argument(
        "-p", "--show-pkg", action="store_true", help="show package for each node"
    )
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # Align with topic view: if hidden topics are included (-i),
    # also show internal nodes (ros2cli daemon + this introspector).
    global HIDE_INTERNAL
    HIDE_INTERNAL = not args.include_hidden_topics

    rclpy.init()
    n = Introspector()
    try:
        time.sleep(0.2)  # allow discovery to settle
        graph = build_node_graph(n, include_hidden=args.include_hidden_topics)
        ns = n.get_namespace() or "/"
        ns = ns if ns.startswith("/") else "/" + ns
    finally:
        rclpy.shutdown()

    # Linux-only process scan + robust mapping (no extra CLI flags)
    procs = find_processes(hide_cli_daemon=HIDE_INTERNAL)
    mapping = map_nodes_to_processes(graph, procs)

    ls_colors = parse_ls_colors() if args.show_color else None

    print(ns if ns != "" else "/")
    print_nodes_tree(
        graph,
        mapping,
        enable_color=args.show_color,
        ls_colors=ls_colors,
        show_type=args.show_msg_type,
        show_qos=args.show_qos,
        node_filter=args.show_node,
        show_pid=args.show_pid,
        show_pkg=args.show_pkg,
    )

    n_nodes, n_topics = compute_node_topic_counts(graph, node_filter=args.show_node)
    print(f"\n{n_nodes} nodes, {n_topics} topics")


if __name__ == "__main__":
    main()
