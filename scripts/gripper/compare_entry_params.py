#!/usr/bin/env python3
"""viz vs headless 两个入口的**行为档位**比对(2026-09-10)。

动机:2026-09-05 查出两个入口的 mhe_tau_source/motor_window_avg 长期不一致 ——
"演示看到的和批次统计的不是同一个估计器输入",而这件事**没有任何机制会报警**,
纯靠人去 grep 才发现。这个脚本把它变成一条命令,改完任一入口都该跑一遍。

用法: python3 src/scripts/gripper/compare_entry_params.py mhe_node
      python3 src/scripts/gripper/compare_entry_params.py acados_nmpc_node

⚠️ 已知的解析局限(读结果时要人工复核这几类):
  · `if/else` 分支里的赋值只取**首次**出现的那个 —— 例如 viz 的 THETA/CALPHA 有
    thetastar 与 M0 两套,脚本会报 thetastar 那套;METHOD 默认 M0,实际与 headless 一致。
  · 只做文本解析、**不执行脚本**(两个入口都含 kill/cleanup,见记忆 sitl-stack-cleanup:
    永不从含 kill 的脚本里抽片段执行)。
  · WORKPOINT 那条正则圈掉的是"实验设定"类参数,漏网的(如 grip_dynamic_after_lift)
    会落进行为档位栏,需要自己判断。

· 未传的参数用节点 declare_parameter 的默认值补齐 —— 不补的话"未传"会被误读成差异
· 工况参数(轨迹/载荷/偏心/时序)单列:两个入口用途不同,这些**本来就该不同**
· 同行多赋值(A=..; B=..)按 ; 切开再解析
"""
import re, sys, pathlib

VIZ='src/scripts/gripper/run_sitl_gripper_viz.sh'; HL='src/scripts/gripper/run_gripper_headless.sh'
NODE={'mhe_node':'src/offboard_test_acados/offboard_test_acados/mhe_node.py',
      'acados_nmpc_node':'src/offboard_test_acados/offboard_test_acados/acados_nmpc_node.py'}
# 工况参数:实验设定,不属于"对齐"范畴
WORKPOINT=re.compile(r'^(grip_(dyn_|payload_mass|x|y|z_|arm_d|lift_|drop_|attach_tol|mass_step|mp_cap)|'
                     r'eval_true_payload_mass|attach_window_sec|residual_log_dir|motor_speed_topic)')

def block(path,node):
    s=pathlib.Path(path).read_text(); i=s.index(f'{node} --ros-args'); out=[]
    for ln in s[i:].splitlines():
        out.append(ln)
        if not ln.rstrip().endswith('\\'): break
    return '\n'.join(out), s

def params(txt):
    d={}
    for m in re.finditer(r"-p\s+([A-Za-z_0-9]+):=", txt):
        k=m.group(1); rest=txt[m.end():]
        if rest.startswith('$('):
            depth=0
            for j,ch in enumerate(rest):
                if ch=='(': depth+=1
                elif ch==')':
                    depth-=1
                    if depth==0: v=rest[:j+1]; break
        elif rest.startswith(("'",'"')):
            q=rest[0]; v=rest[:rest.index(q,1)+1]
        else:
            v=re.match(r'\S+',rest).group(0).rstrip('\\')
        d[k]=v.strip()
    return d

def assigns(script):
    """收集 VAR=值,支持同行多赋值。"""
    out={}
    for ln in script.splitlines():
        ln=ln.split('#')[0]
        for part in ln.split(';'):
            m=re.match(r'\s*([A-Za-z_0-9]+)=(.*)$', part)
            if m and m.group(1) not in out:
                out[m.group(1)]=m.group(2).strip()
    return out

def resolve(v, env, depth=0):
    if depth>8: return v
    v=v.strip().strip('"').strip("'")
    m=re.fullmatch(r'\$\((?:_b|_f2d|_f2dv)\s+(.*)\)', v, re.S)
    if m: return resolve(m.group(1), env, depth+1)
    m=re.fullmatch(r'\$\{([A-Za-z_0-9]+):-(.*)\}', v, re.S)
    if m: return resolve(m.group(2), env, depth+1) if m.group(2) else ''
    m=re.fullmatch(r'\$\{?([A-Za-z_0-9]+)\}?', v)
    if m:
        name=m.group(1)
        return resolve(env[name], env, depth+1) if name in env else f'<{name}?>'
    return v

def node_defaults(path):
    s=pathlib.Path(path).read_text(); d={}
    for m in re.finditer(r"declare_parameter\(\s*'([A-Za-z_0-9]+)'\s*,\s*([^)]+?)\)", s):
        d[m.group(1)]=m.group(2).strip()
    return d

def norm(x):
    x=str(x).strip().strip('"').strip("'").rstrip('\\').strip()
    low=x.lower()
    if low in ('true','1'): return 'true'
    if low in ('false','0','0.0'): return 'false'
    try: return f'{float(x):g}'
    except ValueError: return x.replace(' ','')

def main(node):
    ta,sa=block(VIZ,node); tb,sb=block(HL,node)
    ea,eb=assigns(sa),assigns(sb); nd=node_defaults(NODE[node])
    a={k:norm(resolve(v,ea)) for k,v in params(ta).items()}
    b={k:norm(resolve(v,eb)) for k,v in params(tb).items()}
    keys=sorted(set(a)|set(b)|set())
    beh=[]; wp=[]
    for k in keys:
        va=a.get(k, norm(nd[k]) if k in nd else None)
        vb=b.get(k, norm(nd[k]) if k in nd else None)
        src_a='' if k in a else '(默认)'; src_b='' if k in b else '(默认)'
        if va!=vb:
            (wp if WORKPOINT.match(k) else beh).append((k,va,vb,src_a,src_b))
    print(f"### {node}   行为档位差异 {len(beh)} 项   工况参数差异 {len(wp)} 项(本就该不同)")
    if beh:
        print(f"  {'参数':<30}{'viz':<26}{'headless':<26}")
        for k,va,vb,sa_,sb_ in beh:
            print(f"  {k:<30}{str(va)+sa_:<26}{str(vb)+sb_:<26}")
    else:
        print("  ✓ 行为档位完全一致")
    if wp:
        print(f"\n  --- 工况参数(实验设定,不属于对齐范畴)---")
        for k,va,vb,sa_,sb_ in wp:
            print(f"  {k:<30}{str(va)+sa_:<26}{str(vb)+sb_:<26}")

if __name__=='__main__': main(sys.argv[1])
