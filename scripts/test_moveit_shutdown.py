#!/usr/bin/env python3
"""Start the installed Pi Arm move_group in isolation and require clean shutdown.

Uses mock configuration without a hardware driver or motion requests. Source the
workspace under test first; the selected ROS domain must be unused.
"""
import argparse
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain-id', type=int, default=91, choices=range(0, 233), metavar='ID')
    parser.add_argument('--signal', choices=('SIGINT', 'SIGTERM'), default='SIGINT')
    args = parser.parse_args()
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', PYTHONDONTWRITEBYTECODE='1')
    with tempfile.TemporaryDirectory(prefix='pi-arm-moveit-shutdown-') as directory:
        log_path = Path(directory) / 'move-group.log'
        command = [sys.executable, '-c',
                   'import os,signal,sys; signal.signal(signal.SIGINT,signal.SIG_DFL); '
                   'os.execvp(sys.argv[1],sys.argv[1:])',
                   'ros2', 'launch', 'pi_arm_moveit_config', 'move_group.launch.py',
                   'hardware_type:=mock']
        with log_path.open('w') as log:
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                deadline = time.monotonic() + 45
                while 'You can start planning now!' not in log_path.read_text():
                    if process.poll() is not None or time.monotonic() >= deadline:
                        raise AssertionError('move_group did not finish startup')
                    time.sleep(.1)
                started = re.search(r'\[move_group-\d+\]: process started with pid \[(\d+)\]',
                                    log_path.read_text())
                assert started, 'missing move_group PID'
                # Signal the ROS node itself; launch can exit zero even when this child crashes.
                os.kill(int(started.group(1)), getattr(signal, args.signal))
                code = process.wait(timeout=25)
                output = log_path.read_text()
                assert code == 0, f'launch exited {code}'
                assert re.search(r'\[move_group-\d+\]: process has finished cleanly', output), output
                assert not any(text in output for text in
                               ('Segmentation fault', 'process has died', 'Traceback', 'SIGKILL')), output
            except Exception:
                print(log_path.read_text(), file=sys.stderr)
                raise
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=6)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait(timeout=5)
    print(f'PASS move_group initializes and exits cleanly on {args.signal}')


if __name__ == '__main__':
    main()
