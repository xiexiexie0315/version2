"""Headless Gazebo integration test; uses isolated transport partition."""
import json,os,signal,subprocess,time
from pathlib import Path
ws=Path(__file__).resolve().parents[3]
env=os.environ.copy(); env['IGN_PARTITION']='hnu_joint_test_'+str(os.getpid()); env['IGN_GAZEBO_RESOURCE_PATH']=str(ws/'src/hnu2027_sentry_description/resource/models')
logpath=Path('/tmp/hnu_original_runtime.log'); report=[]
def state():
    r=subprocess.run(['ign','topic','-e','--json-output','-n','1','-t','/model/hnu2027_sentry/joint_state'],env=env,capture_output=True,text=True,timeout=8)
    rows=[json.loads(line) for line in r.stdout.splitlines() if line.startswith('{')]
    assert rows,'No joint feedback'
    return {j['name']:j.get('axis1',{}).get('position',0.0) for j in rows[-1]['joint'] if 'axis1' in j}
def command(name,target):
    subprocess.run(['ign','topic','-t',f'/model/hnu2027_sentry/joint/{name}/cmd_pos','-m','ignition.msgs.Double','-p',f'data: {target}'],env=env,capture_output=True,text=True,timeout=5,check=True)
def reach(targets,label):
    for name,target in targets.items(): command(name,target)
    deadline=time.monotonic()+18
    while time.monotonic()<deadline:
        actual=state()
        if all(abs(actual[name]-target)<0.03 for name,target in targets.items()):
            row=dict(case=label,targets=targets,actual={n:actual[n] for n in targets},passed=True); report.append(row); print(json.dumps(row),flush=True); return
        time.sleep(0.3)
    raise AssertionError(f'{label}: target={targets}, actual={actual}')
with logpath.open('w') as log:
    p=subprocess.Popen(['ign','gazebo','-s','-r','-v','3',str(ws/'src/hnu2027_sentry_simulation/resource/worlds/original_model_debug.sdf')],env=env,stdout=log,stderr=log,start_new_session=True)
    try:
        time.sleep(2)
        initial=state(); assert len(initial)==8,initial
        report.append(dict(case='eight_joint_feedback',passed=True))
        reach(dict(big_yaw=0.3,small_yaw=0.4,pitch=0.15),'gimbal_targets')
        reach(dict(big_yaw=0.0,small_yaw=0.0,pitch=0.0),'gimbal_reset')
        reach(dict(fold_roll=1.57079632679),'fold_90_degrees')
        reach(dict(fold_roll=0.0),'unfold_zero')
        print('PASS: all runtime cases',flush=True)
    finally:
        Path('/tmp/hnu_original_runtime_report.json').write_text(json.dumps(report,indent=2))
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(p.pid,signal.SIGKILL)
