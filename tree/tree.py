# tree.py

from ros2cli.command import CommandExtension
from ros2cli.verb import get_verb_extensions
from ros2cli.verb import VerbExtension as _VerbExtension
from tree.nodes import main as nodes_main
from tree.topics import main as topics_main


class TreeCommand(CommandExtension):
    def add_arguments(self, parser, cli_name):
        self._cmd_parser = parser

        verb_extensions = get_verb_extensions("tree.verb")

        subparsers = parser.add_subparsers(
            dest="verb_name",
            metavar="{nodes,topics}",
            required=False,
        )

        for name, ext in sorted(verb_extensions.items()):
            sp = subparsers.add_parser(name, help="")
            ext.add_arguments(sp, cli_name)

        parser.set_defaults(verb_extensions=verb_extensions)

    def main(self, *, parser, args):
        # Show the same help as ros2 tree -h when no verb is provided
        if not getattr(args, "verb_name", None):
            self._cmd_parser.print_help()
            return 0

        selected = args.verb_name
        return args.verb_extensions[selected].main(parser=self._cmd_parser, args=args)


class NodesVerb(_VerbExtension):
    def add_arguments(self, parser, cli_name=None):
        parser.add_argument("-c", "--show-color", action="store_true")
        parser.add_argument("-m", "--show-msg-type", action="store_true")
        parser.add_argument("-q", "--show-qos", action="store_true")
        parser.add_argument("-i", "--include-hidden-topics", action="store_true")
        parser.add_argument("-N", "--show-node", default=None)
        parser.add_argument("-P", "--show-pid", action="store_true")
        parser.add_argument("-p", "--show-pkg", action="store_true")

    def main(self, *, parser=None, args):
        # Build argv for the nodes entrypoint
        argv = []
        if args.show_color:
            argv.append("--show-color")
        if args.show_msg_type:
            argv.append("--show-msg-type")
        if args.show_qos:
            argv.append("--show-qos")
        if args.include_hidden_topics:
            argv.append("--include-hidden-topics")
        if args.show_node:
            argv += ["--show-node", args.show_node]
        if args.show_pid:
            argv.append("--show-pid")
        if args.show_pkg:
            argv.append("--show-pkg")
        return nodes_main(argv=argv)


class TopicsVerb(_VerbExtension):
    def add_arguments(self, parser, cli_name=None):
        parser.add_argument("-c", "--show-color", action="store_true")
        parser.add_argument("-m", "--show-msg-type", action="store_true")
        parser.add_argument("-q", "--show-qos", action="store_true")
        parser.add_argument("-T", "--show-topic", default=None)
        parser.add_argument("-i", "--include-hidden-topics", action="store_true")
        parser.add_argument("-H", "--show-hz", action="store_true")
        parser.add_argument("--hz-window", type=float, default=1.0)

    def main(self, *, parser=None, args):
        # Build argv for the topics entrypoint
        argv = []
        if args.show_color:
            argv.append("--show-color")
        if args.show_msg_type:
            argv.append("--show-msg-type")
        if args.show_qos:
            argv.append("--show-qos")
        if args.show_topic:
            argv += ["--show-topic", args.show_topic]
        if args.include_hidden_topics:
            argv.append("--include-hidden-topics")
        if args.show_hz:
            argv.append("--show-hz")
        argv += ["--hz-window", str(args.hz_window)]
        return topics_main(argv=argv)
