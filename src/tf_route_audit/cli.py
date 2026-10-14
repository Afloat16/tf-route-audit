import argparse
import sys
import xml.etree.ElementTree as ET
from .common import emit, code
from .core import audit, input_records


def main(argv=None):
    parser = argparse.ArgumentParser(description='Offline TF graph and observed time-window diagnostics.')
    parser.add_argument('input')
    parser.add_argument('--require', action='append', default=[])
    parser.add_argument('--query', nargs=2, metavar=('SOURCE_FRAME', 'TARGET_FRAME'))
    parser.add_argument('--at-ns', type=int)
    parser.add_argument('--strict', action='store_true')
    parser.add_argument('--output')
    args = parser.parse_args(argv)
    try:
        rows, declared = input_records(args.input)
        report = audit(rows, args.require, args.query, args.at_ns, declared)
        emit(report, args.output)
        return code(report, args.strict)
    except (OSError, ValueError, TypeError, KeyError, ET.ParseError) as exc:
        print(f'Input/report error: {exc}', file=sys.stderr)
        return 2
