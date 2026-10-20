#!/usr/bin/env python3
"""Paired control-side CoM ablation; isolated manifest and preserved launch logs."""
import csv, datetime, fcntl, os, pathlib, subprocess, time
WS = pathlib.Path('/home/clear/ros2_ws_HJH')
RES = WS / 'nmpc_test_results'

def main():
    lock = open('/tmp/ecc_ablation.lock', 'w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    # Do not terminate an existing experiment to make room for this one.
    procs = subprocess.check_output(['ps', '-eo', 'comm,args'], text=True)
    if any(('bin/px4' in x or 'run_gripper_headless.sh' in x) and not x.startswith(('python', 'timeout')) for x in procs.splitlines()):
        raise SystemExit('Existing simulator/launcher detected; refusing to start')
    out = RES / ('ecc_ablation_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    out.mkdir()
    env = dict(os.environ, USE_MHE='true', CONTINUOUS_PAYLOAD_ESTIMATES='true',
        NMPC_CONTROL_MODE='mhe', NMPC_GEOM_COUPLED='0', NMPC_GEOM_SOURCE='estimate',
        NMPC_OMEGA_SCALE='true', OMEGA_SCALE_SRC='djest', MHE_C_XY_EST='true',
        NMPC_CXY_FREEZE='false', GRIP_PAYLOAD_KG='0.15', GRIP_ECC_Y='0.10',
        GRIP_BOX_COM_Y='0', GRIP_PRELOADED='false', GRIP_DYNAMIC='true',
        GRIP_DROP_AFTER='55.0', GRIP_DROP_AT_TIP='true', GRIP_DYN_R='10.0',
        GRIP_DYN_W='0.283', GRIP_DYN_RAMP='9.36', GRIP_DYN_DZ='0.8',
        GRIP_Z_HIGH='6.0', GRIP_LIFT_DUR='8.4', ATTACH_WINDOW_SEC='140.0',
        PRESERVE_PX4_LOGS='true')
    import json
    (out / 'settings.json').write_text(json.dumps({k:v for k,v in env.items() if k not in os.environ or os.environ[k]!=v}, indent=2))
    print(out, flush=True)
    with (out / 'manifest.csv').open('w') as mf:
        writer = csv.DictWriter(mf, fieldnames=['idx','wid','arm','stamp','started','finished','status'])
        writer.writeheader(); mf.flush()
        for rep in range(1, int(os.environ.get('PAIRS','4'))+1):
            for arm in (['ON','OFF'] if rep%2 else ['OFF','ON']):
                before=set(RES.glob('grip_nmpc_*.log'))
                started=datetime.datetime.now().isoformat()
                with (out / f'launch_{rep}_{arm}.log').open('w') as lf:
                    launch=subprocess.run(['bash',str(WS/'src/scripts/gripper/run_gripper_headless.sh')], env=dict(env,NMPC_ECC_COMP=str(arm=='ON').lower()),stdout=lf,stderr=subprocess.STDOUT,timeout=360)
                logs=sorted(set(RES.glob('grip_nmpc_*.log'))-before)
                status='no-log'; stamp='NA'
                if logs:
                    log=logs[-1]; stamp=log.stem.removeprefix('grip_nmpc_'); status='timeout'
                    for _ in range(90):
                        text=log.read_text(errors='replace')
                        if 'DROP complete' in text or 'DROP: released' in text:
                            status='ok'; break
                        if 'Traceback (most recent call last)' in text:
                            status='node-error'; break
                        time.sleep(4)
                    if arm=='OFF' and '[ecc-ablation] OFF' not in text: status='invalid-switch'
                writer.writerow(dict(idx=rep,wid='W2',arm=arm,stamp=stamp,started=started,finished=datetime.datetime.now().isoformat(),status=status)); mf.flush()
                print(rep,arm,stamp,status,flush=True)
                if launch.returncode or status in ('no-log','node-error','invalid-switch'):
                    raise SystemExit('Launch/validation failed; inspect logs')
    print('DONE',out,flush=True)
if __name__=='__main__': main()
