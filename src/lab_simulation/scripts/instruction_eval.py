#!/usr/bin/env python3
"""
Planner-only benchmark: plan every instruction case and score the safety check's dry run.

Needs only Ollama with the model, not the simulation.

  ros2 run lab_simulation instruction_eval.py --out /workspace/results
  ros2 run lab_simulation instruction_eval.py --set test --model gemma4:e2b
Writes <out>/<stamp>-planner/{rows.jsonl,summary.json,report.md}.
"""

import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
import urllib.request

from lab_simulation import instruction_cases
from lab_simulation.instruction_eval import evaluate, render_report, summarize
from lab_simulation.planner import DEFAULT_MODEL, DEFAULT_URL, Planner


def model_info(url: str, model: str) -> dict:
    """Return the model's digest and loaded size, as Ollama reports them."""
    info = {}
    try:
        with urllib.request.urlopen(url + '/api/tags', timeout=5) as reply:
            for m in json.load(reply).get('models', []):
                if m.get('name') == model:
                    info['digest'], info['size_bytes'] = m.get('digest'), m.get('size')
        with urllib.request.urlopen(url + '/api/ps', timeout=5) as reply:
            for m in json.load(reply).get('models', []):
                if m.get('name') == model:
                    info['loaded_bytes'], info['vram_bytes'] = m.get('size'), m.get('size_vram')
    except (OSError, ValueError) as error:
        info['error'] = str(error)
    return info


def main(argv=None) -> int:
    """Run the planner-only benchmark."""
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--url', default=DEFAULT_URL)
    p.add_argument('--model', default=DEFAULT_MODEL)
    p.add_argument('--set', choices=('all', 'dev', 'test'), default='all')
    p.add_argument('--out', default='results')
    args = p.parse_args(argv)
    planner = Planner(args.url, args.model)
    problem = planner.check_ready()
    if problem:
        print(problem, file=sys.stderr)
        return 2
    sets = {'dev': instruction_cases.DEV, 'test': instruction_cases.TEST}
    chosen = sets if args.set == 'all' else {args.set: sets[args.set]}
    now = datetime.now()
    out = Path(args.out) / f'{now:%Y%m%d-%H%M%S}-planner'
    out.mkdir(parents=True)
    rows_file = open(out / 'rows.jsonl', 'w')       # written as each case is scored

    def show(row):
        rows_file.write(json.dumps(row, ensure_ascii=False, default=str) + '\n')
        rows_file.flush()
        print(f"{'ok  ' if row['ok'] else 'FAIL'} {row['seconds']:5.1f}s [{row['set']}] "
              f"{row['text'][:60]}" + ('' if row['ok'] else f"  -- {row['why']}"), flush=True)

    rows = []
    with rows_file:
        for name, cases in chosen.items():
            rows += evaluate(planner, cases, name, on_row=show)
    info = {'model': args.model, 'url': args.url, 'date': now.isoformat(timespec='seconds'),
            **model_info(args.url, args.model)}
    summary = summarize(rows)
    (out / 'summary.json').write_text(json.dumps({'info': info, 'summary': summary}, indent=1))
    (out / 'report.md').write_text(render_report(summary, rows, info))
    for name, s in summary.items():
        print(f"{name}: {s['successes']}/{s['n']}  median {s['seconds']['median']:.1f} s")
    print(f'results: {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
