"""Run exact MTC configuration while retaining the original launch parent's lifetime."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

job = Path(sys.argv[1])
runtime = json.loads((job / 'runtime.json').read_text())['mtc']
parent = json.loads((job / 'parent_identity.json').read_text())

def parent_alive():
    try:
        text = Path('/proc/' + str(parent['pid']) + '/stat').read_text()
        fields = text[text.rfind(')') + 2:].split()
        args_now = Path('/proc/' + str(parent['pid']) + '/cmdline').read_bytes().decode().strip('\0').split('\0')
        return fields[0] != 'Z' and fields[19] == parent['start_ticks'] and args_now == parent['args']
    except (FileNotFoundError, ProcessLookupError):
        return False

assert parent_alive(), 'Original ROS launch has already exited'
args = list(runtime['args'])
for i, arg in enumerate(args):
    if arg == '--params-file':
        args[i + 1] = str(job / 'params' / Path(args[i + 1]).name)
for item in Path('/proc').iterdir():
    if not item.name.isdigit():
        continue
    try:
        other_args = (item / 'cmdline').read_bytes().decode().strip('\0').split('\0')
        assert not other_args or other_args[0] != args[0], 'An MTC process is already running'
    except (FileNotFoundError, ProcessLookupError):
        pass
stop_requested = False
def request_stop(signum, frame):
    global stop_requested
    stop_requested = True
signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)
with open(job / 'mtc.log', 'ab', buffering=0) as log:
    child = subprocess.Popen(args, cwd=runtime['cwd'], env=runtime['environment'],
                             stdin=subprocess.DEVNULL, stdout=log, stderr=log)
    record = {'supervisor_pid': os.getpid(), 'child_pid': child.pid,
              'original_launch_pid': parent['pid'], 'original_launch_start_ticks': parent['start_ticks'],
              'policy': 'Send SIGINT only to owned MTC child when original launch exits; never respawn'}
    (job / 'supervisor.json').write_text(json.dumps(record, indent=2))
    sent = False
    while child.poll() is None:
        if (stop_requested or not parent_alive()) and not sent:
            child.send_signal(signal.SIGINT)
            sent = True
        time.sleep(0.1)
    record['child_exit_code'] = child.wait()
    record['sigint_sent_to_child'] = sent
    (job / 'supervisor.json').write_text(json.dumps(record, indent=2))
