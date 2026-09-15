#!/usr/bin/env python3
"""E0 离线回放器(实验计划 §5.2 / §5.7 第 1、3 步)。

把一轮 SITL 的 RESID_LOG_DIR 原始流(motor 250Hz / odom 30Hz / internal 10Hz)
重新喂给**在线同一份** ``MHENode`` 实现,离线复现 MHE 的逐拍估计。

【为什么复用节点类而不是另写一份估计器】
§5.1 要求"外层节点逻辑三臂共用同一实现"。MHE 的质量种子、窗口力平衡重锚、
health gate、释放判据都在 mhe_node.py 里,重写一份就等于引入第二套口径,
保真度门也就失去意义。这里只替换三样东西:
  1. 时钟:``get_clock()`` 换成回放时钟,取值 = 原始流里的 ``t_recv_ns``
     (单调钟)。motor/odom/internal 三流的 recv 都是 MHE 进程内同一个
     ``time.monotonic_ns()``,所以回放不需要任何跨时钟映射;
  2. 调度:不 spin,按 recv 时间归并事件,在 internal 行的时刻调 ``timer_cb``;
  3. 发布:publisher 换成记录器,不连 DDS。

【internal 行与在线 m_est 的对齐】
internal 行在 ``timer_cb`` 通过新鲜度检查之后、``_solve_window`` 之前写入,
记的是**上一拍求解后**的 ``m_est``。所以比较口径是:回放第 k 拍调用
``timer_cb`` 之前的 ``node.m_est`` 对 internal 第 k 行。

【tick 重建】
internal 只在输入新鲜时才写,缺行的 tick 在线上也执行过(可能清空了窗口)。
相邻 internal 间隔 > 1.5·dt 时按 dt 补 tick;首行之前按相位向前补。补出的
tick 若在回放中被判为新鲜,记为 ``phantom_fresh``(在线没写行却被回放当新鲜)
—— 这是保真度诊断量,不应出现。

【参数口径】
在线参数 = 该批次 headless 脚本里 mhe_node 启动段的 ``-p`` + 节点继承的环境。
不手抄映射表:直接从 ``--src-root`` 下的脚本抽出 ``-p`` 行和简单赋值行,交给
bash 按批次的 experiment.env 展开(抽出的片段若含 kill/nohup 等拒绝执行,
见记忆 sitl-stack-cleanup 的 sed 抽片段误杀教训)。

【代码版本】
``--src-root`` 指向哪棵源码树就回放哪份代码。保真度门必须用批次当时的代码
(provenance: HEAD + tracked.patch + untracked.tar.gz 重建);E0 三臂比较用冻结
代码。二者不可混用。

用法(需先 source ROS/工作区并导出 ACADOS 环境,见 run_gripper_headless.sh):
  python3 e0_replay.py --stamp 20260912_235150 \\
      --resid-dir nmpc_test_results/stepab_20260912_234849/B_p1 \\
      --src-root /path/to/reconstructed/src --out /tmp/e0/B_p1
"""
import argparse
import json
import math
import os
import re
import shlex
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

RESULTS_ROOT = Path(__file__).resolve().parents[3] / 'nmpc_test_results'

# 估计器档位(§5.1):在在线配置之上只覆盖这几个环境变量。
ARMS = {
    'online': {},
    'mhe-m': {'MHE_ESTIMATE_MOMENT': '0', 'MHE_GEOM_COUPLED': '0'},
    'mhe-s': {'MHE_ESTIMATE_MOMENT': '1', 'MHE_MOMENT_A_MODE': 'frozen'},
}

_FORBIDDEN = re.compile(r'\b(kill|pkill|killall|nohup|rm|fuser|setsid|ros2|gz|px4)\b')


# --------------------------------------------------------------------------
# 参数:从批次代码树的 headless 脚本展开 mhe_node 的 -p
# --------------------------------------------------------------------------
def _read_env_file(path):
    env = {}
    if path and Path(path).exists():
        for line in Path(path).read_text().splitlines():
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            k, v = line.split('=', 1)
            env[k] = v
    return env


def launch_config(src_root, exp_env):
    """返回 (ros_param_overrides[list of 'k:=v'], node_env[dict])。"""
    script = Path(src_root) / 'scripts/gripper/run_gripper_headless.sh'
    text = script.read_text()
    m = re.search(r'^nohup ros2 run offboard_test_acados mhe_node --ros-args(.*?)'
                  r'> "\$MHE_LOG"', text, re.S | re.M)
    if not m:
        raise RuntimeError(f'找不到 mhe_node 启动段: {script}')
    block = m.group(1)
    params = re.findall(r'-p\s+"?([A-Za-z0-9_]+):=(.*?)"?\s*\\?\n', block)
    params = [(k, v.strip().rstrip('\\').strip().strip('"'))
              for k, v in params]

    # 启动段之前的顶层简单赋值(含 export)。只收 RHS 里没有命令替换、或命令替换
    # 仅为 _f2d/_b 的行 —— 那两个函数是纯字符串换算。
    assigns = []
    prefix = text[:m.start()]
    for line in prefix.splitlines():
        s = line.strip()
        mm = re.match(r'^(export\s+)?([A-Z_][A-Z0-9_]*)=(.*)$', s)
        if not mm:
            continue
        rhs = mm.group(3)
        subs = re.findall(r'\$\((\S+)', rhs)
        if any(x not in ('_f2d', '_b') for x in subs):
            continue
        if '`' in rhs or _FORBIDDEN.search(s):
            continue
        assigns.append(s)

    funcs = [
        """_f2d() { python3 -c "print(float('$1'))"; }""",
        """_b() { case "$(echo "${1:-}" | tr 'A-Z' 'a-z')" in 1|true|yes|on|y) echo true ;; *) echo false ;; esac; }""",
    ]
    lines = ['set -u', 'set +u'] + funcs
    for k, v in exp_env.items():
        lines.append(f'export {k}={shlex.quote(v)}')
    lines += assigns
    for k, v in params:
        lines.append(f'printf "P\\t%s\\t%s\\n" {k} "{v}"')
    lines.append('env | grep -E "^(MHE_|GRIP_|EVAL_)" | sed "s/^/E\\t/"')
    snippet = '\n'.join(lines)
    bad = _FORBIDDEN.findall('\n'.join(f'{k} {v}' for k, v in params))
    if bad:
        raise RuntimeError(f'抽出的片段含禁止命令 {set(bad)},拒绝执行')
    out = subprocess.run(['bash', '-c', snippet], capture_output=True,
                         text=True, env={'PATH': os.environ['PATH'],
                                         'HOME': os.environ.get('HOME', '')},
                         timeout=60)
    if out.returncode != 0:
        raise RuntimeError(f'参数展开失败: {out.stderr[-2000:]}')
    ros_params, node_env = [], {}
    for line in out.stdout.splitlines():
        parts = line.split('\t')
        if parts[0] == 'P' and len(parts) == 3:
            ros_params.append(f'{parts[1]}:={parts[2]}')
        elif parts[0] == 'E' and len(parts) >= 2 and '=' in parts[1]:
            k, v = parts[1].split('=', 1)
            node_env[k] = v
    return ros_params, node_env


# --------------------------------------------------------------------------
# 原始流
# --------------------------------------------------------------------------
def _load_csv(path):
    """容忍被 kill 截断的末行。"""
    with open(path) as f:
        header = f.readline().strip().split(',')
        ncol = len(header)
        rows = []
        for line in f:
            parts = line.rstrip('\n').split(',')
            if len(parts) != ncol:
                continue
            try:
                rows.append([float(x) for x in parts])
            except ValueError:
                continue
    arr = np.array(rows, dtype=float) if rows else np.zeros((0, ncol))
    return {h: arr[:, i] for i, h in enumerate(header)}, len(rows)


def load_streams(resid_dir):
    d = Path(resid_dir)
    out = {}
    for name in ('motor', 'odom', 'internal'):
        files = sorted(d.glob(f'resid_{name}_*.csv'))
        if len(files) != 1:
            raise RuntimeError(f'{d}: resid_{name}_*.csv 应恰有 1 个,实际 {len(files)}')
        out[name], _ = _load_csv(files[0])
    return out


def parse_proximity(stamp, results_root):
    p = Path(results_root) / f'grip_proximity_{stamp}.log'
    info = {'path': str(p), 'attach_wall': [], 'detach_wall': [], 'offset': None}
    if not p.exists():
        return info
    rx = re.compile(r'\[(\d+\.\d+)\].*-> (ATTACH|DETACH) ')
    ro = re.compile(r'attach offset \(box - drone\) = \[([-\d.]+), ([-\d.]+), ([-\d.]+)\]')
    for line in p.read_text(errors='replace').splitlines():
        mm = rx.search(line)
        if mm:
            key = 'attach_wall' if mm.group(2) == 'ATTACH' else 'detach_wall'
            info[key].append(float(mm.group(1)))
            continue
        mo = ro.search(line)
        if mo and info['offset'] is None:
            info['offset'] = [float(mo.group(i)) for i in (1, 2, 3)]
    return info


# --------------------------------------------------------------------------
# 回放
# --------------------------------------------------------------------------
class _ReplayTime:
    __slots__ = ('nanoseconds',)

    def __init__(self, ns):
        self.nanoseconds = int(ns)


class _ReplayClock:
    def __init__(self):
        self.ns = 0

    def now(self):
        return _ReplayTime(self.ns)


class _RecPub:
    def __init__(self, sink, name):
        self.sink, self.name = sink, name

    def publish(self, msg):
        self.sink.append((self.name, msg))


class _SolverProxy:
    """透传 acados solver,截获 solve() 的 status 与耗时。

    在线求解要花几毫秒,``_last_solve_success_sec`` 取的是求解**之后**的时钟,
    所以下一拍的 solution_age≈dt−solve。回放若冻结时钟,age 恰为 dt,连败第 3
    拍会正好压在 payload_solution_fresh_sec=0.30 的门限上,health 判定与在线
    不同。故默认让回放时钟前进本次实测求解耗时(同机串行,量级一致)。
    """

    def __init__(self, solver, clock, advance):
        object.__setattr__(self, '_s', solver)
        object.__setattr__(self, '_clock', clock)
        object.__setattr__(self, '_advance', advance)
        object.__setattr__(self, 'last_status', None)
        object.__setattr__(self, 'last_ms', float('nan'))

    def solve(self):
        t0 = time.perf_counter_ns()
        st = self._s.solve()
        el = time.perf_counter_ns() - t0
        object.__setattr__(self, 'last_ms', el * 1e-6)
        object.__setattr__(self, 'last_status', int(st))
        if self._advance == 'measured':
            self._clock.ns += el
        elif self._advance.startswith('fixed:'):
            self._clock.ns += int(float(self._advance[6:]) * 1e6)
        return st

    def __getattr__(self, k):
        return getattr(self._s, k)


def build_ticks(t_int, dt_ns, t_first_input):
    """internal 行时刻 + 补 tick。返回 (ticks[int ns], row_index[-1=补的])。"""
    ticks, idx = [], []
    if len(t_int):
        t = int(t_int[0]) - dt_ns
        pre = []
        while t > t_first_input:
            pre.append(t)
            t -= dt_ns
        for t in reversed(pre):
            ticks.append(t)
            idx.append(-1)
    for k, t in enumerate(t_int):
        t = int(t)
        if ticks and k > 0 and t - ticks[-1] > 1.5 * dt_ns:
            while ticks[-1] + dt_ns < t - dt_ns // 2:
                ticks.append(ticks[-1] + dt_ns)
                idx.append(-1)
        ticks.append(t)
        idx.append(k)
    return np.array(ticks, dtype=np.int64), np.array(idx, dtype=np.int64)


def run(args):
    src_root = Path(args.src_root).resolve()
    results_root = Path(args.results_root).resolve()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    prov = results_root / f'provenance_{args.stamp}'
    exp_env = _read_env_file(prov / 'experiment.env')
    ros_params, node_env = launch_config(src_root, exp_env)
    # 回放不写原始流(否则会往批次目录里再写一份)。
    ros_params = [p for p in ros_params if not p.startswith('residual_log_dir:=')]
    arm_env = ARMS[args.arm]
    node_env.update(arm_env)
    for k, v in (kv.split('=', 1) for kv in args.env):
        node_env[k] = v
    for kv in args.param:
        k = kv.split(':=', 1)[0]
        ros_params = [p for p in ros_params if not p.startswith(k + ':=')] + [kv]

    # 节点在 import 时读环境(mhe_params):必须先清掉本进程里同名变量再注入。
    for k in list(os.environ):
        if k.startswith(('MHE_', 'GRIP_', 'EVAL_', 'NMPC_')):
            del os.environ[k]
    os.environ.update(node_env)
    # 与在线 SITL 完全隔离:不做任何 DDS 发现。
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'OFF'
    os.environ.setdefault('ROS_DOMAIN_ID', '93')

    pkg_parent = src_root / 'offboard_test_acados'
    # 7da5423 之前的代码树还 import 旧包 offboard_test(quat_to_rotmat),一并放进来。
    legacy = src_root / 'offboard_test'
    if legacy.is_dir():
        sys.path.insert(0, str(legacy))
    sys.path.insert(0, str(pkg_parent))
    import rclpy
    from offboard_test_acados import mhe_node as mn
    from offboard_test_acados.mhe_model import MODEL_NAME as model_name
    if not Path(mn.__file__).resolve().is_relative_to(pkg_parent):
        raise RuntimeError(f'导入到了错误的包: {mn.__file__}(应在 {pkg_parent})')

    streams = load_streams(args.resid_dir)
    mo, od, it = streams['motor'], streams['odom'], streams['internal']
    dt_ns = int(round(mn.mhe_p.dt * 1e9))

    rclpy.init(args=['--ros-args'] + sum((['-p', p] for p in ros_params), []))
    node = mn.MHENode()
    node.timer.cancel()
    clock = _ReplayClock()
    node.get_clock = lambda: clock
    pubs = []
    for attr in ('mass_pub', 'tau_phys_pub', 'c_xy_est_pub',
                 'payload_estimate_pub', 'payload_lost_pub'):
        if hasattr(node, attr):
            setattr(node, attr, _RecPub(pubs, attr))
    node.solver = _SolverProxy(node.solver, clock, args.solve_clock)
    seeds = []
    _orig_seed = node._seed_mass_from_thrust

    def _seed(why, _o=_orig_seed):
        r = _o(why)
        seeds.append((clock.ns, why, float(r[0]), bool(r[1])))
        return r
    node._seed_mass_from_thrust = _seed

    from nav_msgs.msg import Odometry
    from actuator_msgs.msg import Actuators
    om, am = Odometry(), Actuators()

    t_int = it['t_recv_ns']
    t_first = int(min(mo['t_recv_ns'][0], od['t_recv_ns'][0]))
    ticks, row_idx = build_ticks(t_int, dt_ns, t_first)

    # 三类事件归并:0=odom 1=motor 2=tick;同一纳秒先传感器后 tick。
    ev_t = np.concatenate([od['t_recv_ns'], mo['t_recv_ns'], ticks.astype(float)])
    ev_k = np.concatenate([np.zeros(len(od['t_recv_ns'])), np.ones(len(mo['t_recv_ns'])),
                           np.full(len(ticks), 2)])
    ev_i = np.concatenate([np.arange(len(od['t_recv_ns'])), np.arange(len(mo['t_recv_ns'])),
                           np.arange(len(ticks))])
    order = np.lexsort((ev_k, ev_t))

    rows = []
    wall_t0 = time.perf_counter()
    for e in order:
        kind, i = int(ev_k[e]), int(ev_i[e])
        if kind == 0:
            clock.ns = int(od['t_recv_ns'][i])
            p, q = om.pose.pose.position, om.pose.pose.orientation
            p.x, p.y, p.z = od['px'][i], od['py'][i], od['pz'][i]
            q.w, q.x, q.y, q.z = od['qw'][i], od['qx'][i], od['qy'][i], od['qz'][i]
            v, w = om.twist.twist.linear, om.twist.twist.angular
            v.x, v.y, v.z = od['vb_x'][i], od['vb_y'][i], od['vb_z'][i]
            w.x, w.y, w.z = od['wb_x'][i], od['wb_y'][i], od['wb_z'][i]
            node.odom_cb(om)
        elif kind == 1:
            clock.ns = int(mo['t_recv_ns'][i])
            am.velocity = [float(mo['w1'][i]), float(mo['w2'][i]),
                           float(mo['w3'][i]), float(mo['w4'][i])]
            node.motor_speed_cb(am)
        else:
            clock.ns = int(ticks[i])
            k = int(row_idx[i])
            m_before = float(node.m_est)
            frames_before = node.frames
            n_pub = len(pubs)
            node.solver.last_status = None
            node.timer_cb()
            fresh_logged = node.frames > frames_before
            est = None
            for name, msg in pubs[n_pub:]:
                if name == 'payload_estimate_pub':
                    est = list(msg.data)
            s = getattr(node, 's_est', None)
            s = np.asarray(s, dtype=float) if s is not None and len(s) else np.full(2, np.nan)
            rows.append({
                't_ns': int(ticks[i]), 'row': k,
                'online_m': float(it['m_est'][k]) if k >= 0 else math.nan,
                'm_before': m_before, 'm_after': float(node.m_est),
                's_x': float(s[0]), 's_y': float(s[1]),
                'status': (-1 if node.solver.last_status is None
                           else node.solver.last_status),
                'solve_ms': (node.solver.last_ms
                             if node.solver.last_status is not None else math.nan),
                'fresh': int(fresh_logged),
                'phantom_fresh': int(fresh_logged and k < 0),
                'missed_fresh': int((not fresh_logged) and k >= 0),
                'present': int(bool(getattr(node, '_payload_present', False))),
                'latched': int(bool(getattr(node, '_s_release_latched', False))),
                'est': est,
            })
    wall_replay = time.perf_counter() - wall_t0
    node.destroy_node()
    rclpy.shutdown()

    # 墙钟映射:odom header(墙钟) − recv(单调钟) 的中位数(§5.2 时钟对齐规则)。
    off = od['t_header_ns'] - od['t_recv_ns']
    off_med = float(np.median(off))
    off_res = off - off_med
    prox = parse_proximity(args.stamp, results_root)

    def wall_to_mono(tw):
        return tw * 1e9 - off_med

    t_att = wall_to_mono(prox['attach_wall'][0]) if prox['attach_wall'] else None
    t_det = None
    if prox['detach_wall'] and t_att is not None:
        later = [x for x in prox['detach_wall'] if wall_to_mono(x) > t_att]
        t_det = wall_to_mono(later[0]) if later else None

    # 逐拍 CSV
    est_cols = None
    csv_path = out_dir / 'replay_ticks.csv'
    with open(csv_path, 'w') as f:
        base = ['t_ns', 'row', 'online_m', 'm_before', 'm_after', 's_x', 's_y',
                'status', 'solve_ms', 'fresh', 'phantom_fresh', 'missed_fresh',
                'present', 'latched']
        n_est = max((len(r['est']) for r in rows if r['est']), default=0)
        est_cols = [f'est{j}' for j in range(n_est)]
        f.write(','.join(base + est_cols) + '\n')
        for r in rows:
            vals = [r[c] for c in base]
            e = r['est'] or [math.nan] * n_est
            f.write(','.join(f'{v:.9g}' if isinstance(v, float) else str(v)
                             for v in vals + list(e)) + '\n')

    # 保真度门(§5.2):|m_replay − m_online|,按段
    t = np.array([r['t_ns'] for r in rows], dtype=float)
    k = np.array([r['row'] for r in rows])
    d = np.array([abs(r['m_before'] - r['online_m']) if r['row'] >= 0 else math.nan
                  for r in rows])
    seg = {'all': k >= 0}
    if t_att is not None:
        seg['pre_attach'] = (k >= 0) & (t < t_att)
        end_loaded = t_det if t_det is not None else math.inf
        seg['loaded_steady'] = (k >= 0) & (t >= t_att + 5e9) & (t < end_loaded)
    if t_det is not None:
        seg['post_detach'] = (k >= 0) & (t >= t_det)
        seg['post_detach_steady'] = (k >= 0) & (t >= t_det + 5e9)

    def stats(mask):
        x = d[mask]
        x = x[np.isfinite(x)]
        if not len(x):
            return {'n': 0}
        return {'n': int(len(x)), 'p50': float(np.percentile(x, 50)),
                'p95': float(np.percentile(x, 95)), 'max': float(x.max())}

    fid = {name: stats(mask) for name, mask in seg.items()}
    gate_segs = [s for s in ('loaded_steady', 'post_detach') if s in fid]
    # 保真度门只对"与在线同配置"的回放有意义;换档或加覆盖后 fidelity 只是
    # "与在线差多少"的描述量,gate_pass 记为 None。
    gate_applicable = args.arm == 'online' and not args.env and not args.param
    gate_pass = (bool(gate_segs) and all(
        fid[s].get('n', 0) > 0 and fid[s]['p95'] <= args.gate_kg for s in gate_segs)
    ) if gate_applicable else None
    first_div = None
    big = np.where(np.isfinite(d) & (d > args.gate_kg))[0]
    if len(big):
        j = int(big[0])
        first_div = {'t_rel_s': (t[j] - t[0]) * 1e-9, 'row': int(k[j]),
                     'online_m': rows[j]['online_m'], 'replay_m': rows[j]['m_before'],
                     'rel_attach_s': ((t[j] - t_att) * 1e-9 if t_att is not None else None)}
    st = np.array([r['status'] for r in rows])
    summary = {
        'stamp': args.stamp, 'resid_dir': str(args.resid_dir), 'arm': args.arm,
        'src_root': str(src_root), 'module': mn.__file__,
        'model_name': model_name,
        'ros_params': ros_params, 'node_env': node_env,
        'n_ticks': len(rows), 'n_internal_rows': int(len(t_int)),
        'phantom_fresh': int(sum(r['phantom_fresh'] for r in rows)),
        'missed_fresh': int(sum(r['missed_fresh'] for r in rows)),
        'solves': int((st >= 0).sum()), 'solve_fail': int((st > 0).sum()),
        'seed_calls': [{'t_rel_s': (s0 - t[0]) * 1e-9, 'why': w, 'm': m, 'from_thrust': ft}
                       for s0, w, m, ft in seeds],
        'clock_map': {'offset_median_ns': off_med,
                      'residual_ms_p05_p50_p95': [float(np.percentile(off_res, q)) * 1e-6
                                                  for q in (5, 50, 95)]},
        'truth': {'attach_mono_ns': t_att, 'detach_mono_ns': t_det,
                  'attach_offset': prox['offset'], 'proximity_log': prox['path']},
        'solve_clock': args.solve_clock,
        'fidelity': fid, 'gate_kg': args.gate_kg, 'gate_segments': gate_segs,
        'gate_pass': gate_pass, 'first_divergence': first_div,
        'replay_wall_s': wall_replay,
    }
    (out_dir / 'replay_summary.json').write_text(
        json.dumps(summary, indent=2, ensure_ascii=False))
    print(json.dumps({k: summary[k] for k in (
        'stamp', 'arm', 'n_ticks', 'n_internal_rows', 'phantom_fresh', 'missed_fresh',
        'solves', 'solve_fail', 'fidelity', 'gate_pass', 'first_divergence',
        'replay_wall_s')}, indent=2, ensure_ascii=False))
    return 2 if gate_pass is False else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stamp', required=True, help='NMPC 轮次 stamp(proximity/provenance 用)')
    ap.add_argument('--resid-dir', required=True)
    ap.add_argument('--src-root', required=True,
                    help='源码树根(含 offboard_test_acados/ 与 scripts/)')
    ap.add_argument('--results-root', default=str(RESULTS_ROOT))
    ap.add_argument('--arm', choices=sorted(ARMS), default='online')
    ap.add_argument('--env', action='append', default=[], help='额外覆盖 K=V')
    ap.add_argument('--param', action='append', default=[], help='额外覆盖 name:=value')
    ap.add_argument('--gate-kg', type=float, default=0.01)
    ap.add_argument('--solve-clock', default='measured',
                    help="求解期间回放时钟: measured(默认)|zero|fixed:<ms>")
    ap.add_argument('--out', required=True)
    return run(ap.parse_args())


if __name__ == '__main__':
    sys.exit(main())
