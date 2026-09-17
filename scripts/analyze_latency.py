#!/usr/bin/env python3
"""Analyze command or interpolated motion against feedback without starting ROS."""
import argparse
import csv
import json
from pathlib import Path

from dual_crx_control.latency_analysis import estimate_delay, statistics


def analyze(path, args):
    with path.open() as stream:
        rows = list(csv.DictReader(stream))
    results = {}
    input_sources = {}
    for arm in sorted({r['arm'] for r in rows}):
        arm_rows = [row for row in rows if row['arm'] == arm]
        feedback = [row for row in arm_rows if row['source'] == 'feedback']
        input_source = next(
            (source for source in ('command', 'interpolated')
             if any(row['source'] == source for row in arm_rows)),
            None,
        )
        if input_source is None or not feedback:
            continue
        input_sources[arm] = input_source
        command = [row for row in arm_rows if row['source'] == input_source]
        for j in range(1, 7):
            name = f'J{j}_rad'
            try:
                series = [
                    [(float(row['time_s']), float(row[name])) for row in samples]
                    for samples in (command, feedback)
                ]
                ct, cq = zip(*series[0])
                ft, fq = zip(*series[1])
                results[f'{arm}_J{j}'] = estimate_delay(ct, cq, ft, fq,
                    max_lag=args.max_lag, step=args.step, trim_start=args.trim_start,
                    trim_end=args.trim_end)
            except ValueError as exc:
                results[f'{arm}_J{j}'] = {'unavailable': str(exc)}
    scaling = {arm: statistics([float(r['scaling']) for r in rows
                              if r['arm'] == arm and r.get('scaling', '') != ''])
               for arm in sorted({r['arm'] for r in rows})}
    return {'file': str(path.resolve()), 'input_sources': input_sources,
            'joints': results, 'scaling': scaling,
            'interpretation': 'Positive delay means feedback lags the selected input; phase delay includes feedback transport. '
                              'Receipt timestamps are not robot execution timestamps.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-lag', type=float, default=.5)
    parser.add_argument('--step', type=float, default=.001)
    parser.add_argument('--trim-start', type=float, default=1.5)
    parser.add_argument('--trim-end', type=float, default=1.)
    args = parser.parse_args()
    reports = [analyze(path, args) for path in args.csv]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(reports, indent=2) + '\n')
    for report in reports:
        print(Path(report['file']).name)
        for joint, result in report['joints'].items():
            if 'delay_ms' in result:
                print(f"  {joint}: {result['delay_ms']:.1f} ms, r={result['correlation']:.5f}, gain={result['gain']:.4f}")
            else:
                print(f"  {joint}: {result['unavailable']}")


if __name__ == '__main__':
    main()
