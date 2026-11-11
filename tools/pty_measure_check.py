#!/usr/bin/env python3
"""measure の CLI を、偽サーボ（pty）と、制御端末つきの pty で動かして確認する。実機・実ポートは開かない。

  python3 tools/pty_measure_check.py <so_arm101_calib の実行ファイル> <setup.bash> new|old <作業ディレクトリ> [記録の秒数]

stdout/stderr はパイプ（tee につないだときと同じ）。入力と /dev/tty は pty。
new: Enter は 中央・終了 の 2 回（中央の測定中にも余計な Enter を 1 回送り、入力の破棄を確認する）。
old: Enter は 中央・記録開始・終了 の 3 回（改善前）。
"""
import json
import os
import pty
import re
import select
import subprocess
import sys
import time

exe, setup, mode, work = sys.argv[1:5]
secs = float(sys.argv[5]) if len(sys.argv) > 5 else 6.0
repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
yaml_path = repo + '/src/so_arm101_hardware/config/follower_calibration.yaml'
state = os.path.join(work, 'state')
os.makedirs(state, exist_ok=True)

fake = subprocess.Popen(['python3', repo + '/tools/fake_sts3215.py', '--calibration', yaml_path, '--state-dir', state],
                        stdout=subprocess.PIPE, text=True)
port = fake.stdout.readline().strip()

BASE = {1: 2000, 2: 2000, 3: 2000, 4: 2000, 5: 2000, 6: 2000}
LO = {1: -1300, 2: -1100, 3: -1000, 4: -1200, 5: -1800, 6: -700}
HI = {1: 1300, 2: 1100, 3: 1000, 4: 1200, 5: 1800, 6: 700}


def set_pos(d):
    tmp = os.path.join(state, 'positions.json.tmp')
    with open(tmp, 'w') as f:
        json.dump({str(k): v for k, v in d.items()}, f)
    os.replace(tmp, os.path.join(state, 'positions.json'))


def reads():
    try:
        return json.load(open(os.path.join(state, 'instructions.json'))).get('0x82', 0)
    except (OSError, ValueError):
        return 0


set_pos(BASE)
backup = os.path.join(work, 'backup.json')
subprocess.run(['bash', '-c', f'source {setup}; {exe} backup --port {port} --calibration {yaml_path} --out {backup}'],
               check=True, capture_output=True)

r_pipe, w_pipe = os.pipe()
pid, master = pty.fork()
if pid == 0:
    os.dup2(w_pipe, 1)
    os.dup2(w_pipe, 2)
    os.close(r_pipe)
    out = os.path.join(work, 'measure_out.json')
    os.execvp('bash', ['bash', '-c',
                       f'source {setup}; exec {exe} measure --port {port} --calibration {yaml_path} '
                       f'--backup {backup} --out {out}'])
os.close(w_pipe)

pipe_buf, tty_buf = b'', b''
t_start = time.time()


def pump(timeout=0.05):
    global pipe_buf, tty_buf
    rl, _, _ = select.select([r_pipe, master], [], [], timeout)
    for fd in rl:
        try:
            data = os.read(fd, 65536)
        except OSError:
            data = b''
        if fd == r_pipe:
            pipe_buf += data
        else:
            tty_buf += data


def wait_for(text, limit=15.0):
    end = time.time() + limit
    while time.time() < end:
        pump()
        if text.encode() in pipe_buf:
            return True
    return False


def send_enter():
    os.write(master, b'\n')


assert wait_for('>>> 腕を「中央」'), pipe_buf.decode(errors='replace')
send_enter()                                  # 中央の Enter
t_rec = None
if mode == 'new':
    time.sleep(0.05)
    send_enter()                              # 中央の測定中に押された余計な Enter（破棄されるはず）
    assert wait_for('中央の測定 OK、記録中'), pipe_buf.decode(errors='replace')
else:
    assert wait_for('>>> 全関節を端から端まで'), pipe_buf.decode(errors='replace')
    send_enter()                              # 記録開始の Enter
t_rec = time.time()
n0 = reads()
# 記録中: 動かす。pan は前回の端（yaml）を超えるまで押す。他は範囲いっぱい
steps = int(secs * 2)
for k in range(steps):
    cur = dict(BASE)
    ph = k / max(steps - 1, 1)
    for i in cur:
        span = (LO[i], HI[i])
        cur[i] = BASE[i] + int(span[0] + (span[1] - span[0]) * (0.5 - 0.5 * __import__('math').cos(2 * 3.14159 * ph)))
    set_pos(cur)
    end = time.time() + 0.5
    while time.time() < end:
        pump(0.05)
t_end = time.time()
n1 = reads()
send_enter()                                  # 終了の Enter
end = time.time() + 5
while time.time() < end:
    pump()
    try:
        done, status = os.waitpid(pid, os.WNOHANG)
    except ChildProcessError:
        done, status = pid, 0
    if done:
        break
pump(0.2)
if mode == 'new':
    # 測った新しい形式（schema 2）のファイルを、偽サーボに対して apply の dry-run（何も書かない）に通す
    res = subprocess.run(['bash', '-c', f'source {setup}; {exe} apply --port {port} --calibration {yaml_path} '
                          f'--measure {os.path.join(work, "measure_out.json")} --backup {backup}'],
                         capture_output=True, text=True)
    print('--- apply の dry-run（schema 2）: exit =', res.returncode)
    print('\n'.join(l for l in res.stdout.splitlines() if 'dry-run' in l or 'ERROR' in l or '旧形式' in l))
    print('書き込み回数（writes.jsonl の行数）:', sum(1 for _ in open(os.path.join(state, 'writes.jsonl'))))
fake.terminate()

log = pipe_buf.decode(errors='replace')
tty = tty_buf.decode(errors='replace')
dur = t_end - t_rec
print(f'mode={mode}  exit={os.WEXITSTATUS(status) if done else "running"}')
print(f'記録の実時間 {dur:.2f} s、SYNC_READ {n1 - n0} 回、{(n1 - n0) / dur:.1f} 回/秒（間隔 {1000 * dur / max(n1 - n0, 1):.1f} ms）')
print('stdout（ログ）に ESC が含まれる:', '\x1b' in log)
print('端末（/dev/tty）に出た表示の長さ:', len(tty), 'bytes、カーソル上移動（ESC[7A）の回数:', tty.count('\x1b[7A'))
print('ログの 1 秒ごとの行（[N s]）の数:', len(re.findall(r'\[\d+ s\] ', log)))
m = re.search(r'記録: .*', log)
print('統計:', m.group(0) if m else '(なし。改善前)')
# 最初の記録が即終了していないか（記録の統計の読み取り回数が実時間に見合うか）
print('--- ログの抜粋 ---')
for ln in log.splitlines():
    if re.search(r'\[\d+ s\] shoulder_pan|記録中|記録:|\| ?\d+|exit|ERROR|OK', ln):
        print(ln)
if mode == 'new':
    last = tty.split('\x1b[7A')[-1]
    print('--- 端末の最後のフレーム ---')
    print(re.sub(r'\x1b\[K|\r', '', last))
