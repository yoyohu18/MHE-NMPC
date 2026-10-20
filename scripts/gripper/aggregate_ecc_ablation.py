#!/usr/bin/env python3
"""Report all attempts, paired pre-drop metrics and exploratory uncertainty."""
import csv, pathlib, sys, math, statistics, itertools, re
from aggregate_l1_ab import parse_run
folder=pathlib.Path(sys.argv[1]); res=folder.parent
rows=[]
for row in csv.DictReader((folder/'manifest.csv').open()):
    if row['stamp']=='NA':
        rows.append(row); continue
    row.update(parse_run(res/f"grip_nmpc_{row['stamp']}.log") | {'run_status':row['status']})
    prox=res/f"grip_proximity_{row['stamp']}.log"
    state=res/f"grip_state_{row['stamp']}.log"
    row['physical_attach']='ATTACHED ' in state.read_text(errors='replace') if state.exists() else False
    row['physical_detach']='DETACHED ' in state.read_text(errors='replace') if state.exists() else False
    text=(res/f"grip_nmpc_{row['stamp']}.log").read_text(errors='replace')
    matches=re.findall(r'g0=([-+0-9.eE]+) g1=([-+0-9.eE]+) g2=([-+0-9.eE]+)',text)
    row['solver_c_peak']=max((math.hypot(float(y),float(z)) for x,y,z in matches),default=math.nan)
    row['solver_dj_peak']=max((float(x) for x,y,z in matches),default=math.nan)
    row['switch_valid']=bool(matches) and (row['solver_c_peak']==0 if row['arm']=='OFF' else row['solver_c_peak']>0)
    offsets=re.findall(r'\[EVT\] name=attach_offset.*rx=([-+0-9.eE]+) ry=([-+0-9.eE]+) rz=([-+0-9.eE]+)',prox.read_text(errors='replace') if prox.exists() else '')
    if offsets:
        row.update(dict(zip(['rx','ry','rz'],map(float,offsets[0]))))
    if not row['physical_attach'] or not row['switch_valid']:
        row['run_status']='invalid-attach-or-switch'
    rows.append(row)
cols=list(dict.fromkeys(k for r in rows for k in r))
with (folder/'metrics.csv').open('w') as f:
    w=csv.DictWriter(f,fieldnames=cols); w.writeheader(); w.writerows(rows)
lines=['# 偏心控制补偿消融（探索性）','',
'仅将 NMPC 模型横向质心 c_xy 置零；保留质量、dJ、内环补偿、MHE 和释放判据。',
'工况：0.15 kg、名义抓取偏心 0.10 m、8 字轨迹。相邻两次为时间配对，物理吸附偏移并不保证一致。',
'当前源码含既有未提交修改，各轮 provenance 保存实际代码；不能声称运行于某个干净 commit。','',
'|配对|臂|执行|飞行判定|起吊 RMS (m)|8字 RMS (m)|8字峰值 (m)|',
'|---|---|---|---|---|---|---|']
def fmt(x):
    if isinstance(x, bool): return 'yes' if x else 'no'
    return f'{x:.4f}' if isinstance(x,(int,float)) and math.isfinite(x) else str(x)
for r in rows:
    lines.append('|'+ '|'.join(fmt(r.get(k,'NA')) for k in ['idx','arm','run_status','status','lift_rms','fig8_rms','fig8_peak'])+'|')
lines+=['','|配对|臂|真实吸附|rx (m)|ry (m)|rz (m)|模型 c 峰值 (m)|开关有效|','|---|---|---|---|---|---|---|---|']
for r in rows:
    lines.append('|'+ '|'.join(fmt(r.get(k,'NA')) for k in ['idx','arm','physical_attach','rx','ry','rz','solver_c_peak','switch_valid'])+'|')
lines+=['','配对差值定义为 OFF−ON，正值表示关闭补偿后误差变大。所有尝试保留在明细，未完成或失稳不作为成功配对。']
for metric in ['lift_rms','fig8_rms','fig8_peak']:
    pairs=[]
    for idx in sorted(set(r['idx'] for r in rows)):
        d={r['arm']:r for r in rows if r['idx']==idx}
        if set(d)!={'ON','OFF'}: continue
        if any(r.get('status')!='ok' or r.get('run_status')!='ok' for r in d.values()): continue
        a,b=d['ON'].get(metric,math.nan),d['OFF'].get(metric,math.nan)
        if math.isfinite(a) and math.isfinite(b): pairs.append((a,b))
    if not pairs: continue
    ds=[b-a for a,b in pairs]; mean=statistics.mean(ds)
    # Exact sign-flip test, exploratory with few pairs; no equivalence claim.
    p=sum(abs(statistics.mean(s*x for s,x in zip(sign,ds)))>=abs(mean)-1e-12 for sign in itertools.product([-1,1],repeat=len(ds)))/2**len(ds)
    pct=100*mean/statistics.mean(a for a,b in pairs)
    lines += [f'- {metric}: n={len(ds)}, ON={statistics.mean(a for a,b in pairs):.4f} m, OFF={statistics.mean(b for a,b in pairs):.4f} m，差值={mean:+.4f} m ({pct:+.1f}%)，探索性双侧符号翻转 p={p:.3f}。']
lines+=['','少量重复且未预设等效界限，差异不显著不能解释为等效。论文核心贡献的判断还须结合失败率、实际吸附偏心及补偿幅值审查。']
(folder/'report.md').write_text('\n'.join(lines)+'\n')
print('\n'.join(lines))
