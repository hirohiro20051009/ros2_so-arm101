import json, os, subprocess, sys, time
exe, setup, work = sys.argv[1:4]
repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
yaml_path = repo + '/src/so_arm101_hardware/config/follower_calibration.yaml'
state = os.path.join(work, 'state'); os.makedirs(state, exist_ok=True)
fake = subprocess.Popen(['python3', repo + '/tools/fake_sts3215.py', '--calibration', yaml_path, '--state-dir', state], stdout=subprocess.PIPE, text=True)
port = fake.stdout.readline().strip()
def set_pos(d):
    json.dump({str(k): v for k, v in d.items()}, open(os.path.join(state, 'positions.json'), 'w'))
base = {i: 2000 for i in range(1, 7)}
set_pos(base)
backup = os.path.join(work, 'backup.json')
subprocess.run(['bash', '-c', f'source {setup}; {exe} backup --port {port} --calibration {yaml_path} --out {backup}'], check=True, capture_output=True)
# 制御端末なし（start_new_session=True）、stdin はパイプ
p = subprocess.Popen(['bash', '-c', f'source {setup}; exec {exe} measure --port {port} --calibration {yaml_path} --backup {backup} --out {work}/m.json'],
                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True, text=True)
time.sleep(1.5); p.stdin.write('\n'); p.stdin.flush()
time.sleep(1.0)
for k in range(8):
    set_pos({i: 2000 + int((-1000 if k % 2 == 0 else 1000)) for i in range(1, 7)}); time.sleep(0.3)
p.stdin.write('\n'); p.stdin.flush()
out, _ = p.communicate(timeout=20)
fake.terminate()
print('exit =', p.returncode)
for ln in out.splitlines():
    if any(k in ln for k in ('注意', '記録中', '記録を終えた', 'min 端まで', '記録:', 'ERROR')) or ln.startswith(('shoulder_pan', 'gripper')):
        print(ln)
print('ESC を含む:', '\x1b' in out)
