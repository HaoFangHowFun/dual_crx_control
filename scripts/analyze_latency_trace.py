#!/usr/bin/env python3
"""Localize apparent signal delays using same-host application and driver CSVs."""
import argparse
import csv
import json
from pathlib import Path

from dual_crx_control.latency_analysis import estimate_delay, statistics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--application', type=Path, required=True)
    parser.add_argument('--observer', type=Path)
    parser.add_argument('--driver-dir', type=Path, required=True)
    parser.add_argument('--driver-pid', type=int, required=True,
                        help='one controller_manager PID; each arm has a separate process')
    parser.add_argument('--arm', choices=['left', 'right'], required=True)
    parser.add_argument('--joint', type=int, choices=range(1, 7), default=1)
    parser.add_argument('--trim-start', type=float, default=3.)
    parser.add_argument('--trim-end', type=float, default=1.)
    parser.add_argument('--max-lag', type=float, default=.5)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    stages, driver_rows = {}, []
    def read(path):
        with path.open() as stream:
            return list(csv.DictReader(stream))
    for row in read(args.application):
        if row['arm'] == args.arm:
            name = {'command': 'app_publish', 'generated': 'app_generated',
                    'feedback': 'app_feedback', 'publish_return': 'app_publish_return'}.get(row['source'])
            if name:
                stages.setdefault(name, []).append(row)
    if args.observer:
        for row in read(args.observer):
            if row['arm'] == args.arm:
                stages.setdefault('observer_' + row['source'], []).append(row)
    files = sorted(args.driver_dir.glob(f'*_{args.driver_pid}_*.csv'))
    for path in files:
        for row in read(path):
            stages.setdefault(row['stage'], []).append(row)
            driver_rows.append(row)
    if 'app_publish' not in stages or not files:
        parser.error('missing application commands or driver trace files for that PID')
    lo = min(float(r['time_s']) for r in stages['app_publish'])
    hi = max(float(r['time_s']) for r in stages['app_publish'])
    # Reject mixing old driver sessions/clock epochs rather than extrapolating.
    stages = {k: sorted([r for r in rows if lo <= float(r['time_s']) <= hi],
                        key=lambda r: float(r['time_s'])) for k, rows in stages.items()}
    name = f'J{args.joint}_rad'
    results = {}
    for before, after in [('app_generated', 'app_publish'), ('app_publish', 'hardware_write'),
                          ('hardware_write', 'client_enqueue'), ('client_enqueue', 'send_begin'),
                          ('send_begin', 'status_return'), ('status_return', 'hardware_read'),
                          ('hardware_read', 'observer_feedback'), ('hardware_read', 'app_feedback'),
                          ('app_publish', 'app_feedback'), ('app_publish', 'observer_feedback')]:
        a, b = stages.get(before, []), stages.get(after, [])
        try:
            results[f'{before}->{after}'] = estimate_delay(
                [float(r['time_s']) for r in a], [float(r[name]) for r in a],
                [float(r['time_s']) for r in b], [float(r[name]) for r in b],
                max_lag=args.max_lag, trim_start=args.trim_start, trim_end=args.trim_end)
        except ValueError as exc:
            results[f'{before}->{after}'] = {'unavailable': str(exc)}
    send_rows = stages.get('send_end', [])
    diagnostics = {key: statistics([float(r[key])*scale for r in send_rows])
                   for key, scale in [('queue_depth', 1), ('represented_age_s', 1000),
                                      ('call_duration_s', 1000), ('alpha', 1), ('scaling', 1)]}
    # Units explicitly renamed after converting seconds to milliseconds.
    diagnostics['represented_age_ms'] = diagnostics.pop('represented_age_s')
    diagnostics['send_call_duration_ms'] = diagnostics.pop('call_duration_s')
    intervals = {k: statistics([(float(b['time_s'])-float(a['time_s']))*1000
                                for a, b in zip(rows, rows[1:])]) for k, rows in stages.items()}
    status = stages.get('status_return', [])
    seq = [int(float(r['controller_sequence'])) for r in status]
    diagnostics['status_sequence_nonunit_steps'] = sum(b-a != 1 for a,b in zip(seq,seq[1:]))
    report = dict(arm=args.arm, joint=args.joint, files=[str(p) for p in files],
                  delays=results, driver_statistics=diagnostics, interval_ms=intervals,
                  notes=['Stage delays are waveform phase estimates, not packet round trips.',
                         'send_begin/status_return includes network, controller, servo and feedback delay.',
                         'represented_age uses the existing driver high_resolution_clock timeline; clock jumps can affect it.',
                         'controller_stamp_raw has no assumed unit or synchronization to host clock.',
                         'status sequence attached to send markers identifies the preceding status, not an execution acknowledgement.'])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
