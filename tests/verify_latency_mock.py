#!/usr/bin/env python3
"""Exercise the installed diagnostic sine and passive recorder on local domain 181."""
import csv
import json
import os
from pathlib import Path
import signal
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT/'test_results/latency/mock'


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID='181', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               ROS_STATIC_PEERS='', OPENBLAS_NUM_THREADS='1', ROS_LOG_DIR=str(OUTPUT/'ros_logs'))
    processes = []
    handles = []
    def start(name, command, stdin=None):
        handle=(OUTPUT/(name+'.log')).open('w')
        handles.append(handle)
        process=subprocess.Popen(command,env=env,stdin=stdin,stdout=handle,
                                 stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
        return process
    try:
        start('bringup',['ros2','launch','dual_crx_control','dual_cartesian_mock.launch.py','rviz:=false'])
        observer=start('observer',['ros2','run','dual_crx_control','record_latency.py',
                                  '--duration','13','--output',str(OUTPUT/'observer.csv')])
        sine=start('sine',['ros2','run','dual_crx_control','dual_test2_periodic.py',
                          '--robot-namespace','left','--joint','1','--amplitude','1',
                          '--period','4','--time','8','--initial-hold','1','--ramp-time','1',
                          '--latency-csv',str(OUTPUT/'application.csv'),
                          '--plot-file',str(OUTPUT/'response.png')],stdin=subprocess.PIPE)
        # ENTER is provided only inside this explicitly isolated software mock.
        sine.communicate(input=b'\n',timeout=20)
        assert sine.returncode == 0, (OUTPUT/'sine.log').read_text()
        assert observer.wait(timeout=10) == 0
        result=subprocess.run(['ros2','run','dual_crx_control','analyze_latency.py',
                               str(OUTPUT/'application.csv'),str(OUTPUT/'observer.csv'),
                               '--output',str(OUTPUT/'analysis.json')],env=env,
                              check=True,capture_output=True,text=True)
        print(result.stdout)
        reports=json.loads((OUTPUT/'analysis.json').read_text())
        for report in reports:
            lag=report['joints']['left_J1']['delay_ms']
            assert abs(lag)<30., report
        with (OUTPUT/'application.csv').open() as f:
            rows=list(csv.DictReader(f))
        assert {'generated','command','publish_return','feedback'} <= {r['source'] for r in rows}
        print('Installed diagnostic sine, passive recorder and offline analysis passed in mock.')
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
                try:
                    process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait()
        for handle in handles:
            handle.close()


if __name__ == '__main__':
    main()
