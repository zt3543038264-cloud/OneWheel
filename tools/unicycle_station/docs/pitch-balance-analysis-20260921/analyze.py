from pathlib import Path
import json,hashlib,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
root=Path.cwd();out=root/'tools/unicycle_station/docs/pitch-balance-analysis-20260921';out.mkdir(exist_ok=True)
base=Path('C:/Users/kongmeng/Documents/EmbeddedStation/sessions');name='20260921_131618_332100';p=base/name
raw=(p/'frames.jsonl').read_bytes();records=[]
for l in raw.decode('utf-8').splitlines():
 try:r=json.loads(l)
 except ValueError:continue
 if r.get('kind')=='frame' and not r.get('error'):records.append(r)
stat=[r for r in records if r['tag']=='stat'];att=np.array([[r['time']]+r['values'] for r in records if r['tag']=='att']);run=np.array([[r['time']]+r['values'] for r in records if r['tag']=='run'])
segments=[];start=None
for r in stat:
 active=r['values'][1]==2 and r['values'][2:5]==[0,0,0]
 if active and start is None:start=r['time']
 if not active and start is not None:segments.append((start,r['time']));start=None
keys=['imu_forward_dir','motor_dir_c','enc_dir_c','pitch_zero_init','odom_counts_per_m','p_vel_kp','p_vel_ki','p_vel_kd','p_angle_kp','p_angle_ki','p_angle_kd','p_rate_kp','p_rate_ki','p_rate_kd']
summary={'source':str(p),'frames_sha256':hashlib.sha256(raw).hexdigest(),'transport':json.loads((p/'session.json').read_text(encoding='utf-8'))['transport'],'notes':['Run stream has no Pitch target/rate/output/acceleration diagnostic fields.','PC-time alignment; att is 10 Hz and run has gaps. Physical assistance and wheel slip are not recorded.','Ranges describe encoder-derived speed and estimated attitude, not independent motion capture.'],'episodes':[]}
series=[]
for i,(a,b) in enumerate(segments,1):
 x=run[(run[:,0]>=a)&(run[:,0]<b)];y=att[(att[:,0]>=a)&(att[:,0]<b)]
 params={}
 for r in records:
  if r['time']>a:break
  if r['tag']=='par':params[r['values'][1]]=float(r['values'][3])
 q={k:params.get(k) for k in keys}
 ep={'id':i,'start_pc_seconds':a,'end_pc_seconds':b,'duration_seconds':b-a,'parameters_last_observed_before_start':q,'run_samples':len(x),'att_samples':len(y),'speed_min_mps':float(x[:,21].min()),'speed_max_mps':float(x[:,21].max()),'pitch_min_deg':float(y[:,2].min()),'pitch_max_deg':float(y[:,2].max()),'max_abs_roll_deg':float(abs(x[:,2]).max()),'roll_output_limit_fraction':float(np.mean(abs(x[:,7])>=9999)),'max_run_gap_seconds':float(np.diff(x[:,0]).max())}
 summary['episodes'].append(ep);series.append((a,b,x,y,q))
 with (out/f'episode-{i:02d}.csv').open('w',encoding='utf-8',newline='') as f:
  w=csv.writer(f);w.writerow(['relative_pc_time_s','mcu_uptime_ms','speed_plan_mps','speed_ramp_mps','encoder_speed_mps','roll_deg','roll_target_deg','roll_output','lean_deg','nearest_pitch_deg','att_minus_run_pc_time_s'])
  for r in x:
   idx=int(np.argmin(abs(y[:,0]-r[0])));diff=float(y[idx,0]-r[0]);pitch=y[idx,2] if abs(diff)<=.16 else ''
   w.writerow([r[0]-a,r[1],r[19],r[20],r[21],r[2],r[3],r[7],r[8],pitch,diff])
(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
font=Path('C:/Windows/Fonts/msyh.ttc')
if font.exists():plt.rcParams['font.family']=FontProperties(fname=str(font)).get_name()
plt.rcParams['axes.unicode_minus']=False
fig,axes=plt.subplots(3,2,figsize=(12,8.4),sharex='col',layout='constrained')
for col,num in enumerate([3,10]):
 a,b,x,y,q=series[num-1]
 axes[0,col].plot(y[:,0]-a,y[:,2],'.-',lw=1.4,label='Pitch 估计角')
 axes[0,col].axhline(q['pitch_zero_init'],ls='--',color='gray',label='机械零点设置')
 axes[0,col].set_title(f'第 {num} 次 Balance｜速度 Kp={q["p_vel_kp"]:.3f}，角度 Kp={q["p_angle_kp"]:.0f}')
 axes[0,col].set_ylabel('Pitch / °');axes[0,col].legend(fontsize=9)
 axes[1,col].plot(x[:,0]-a,x[:,21],color='#0e7490',label='编码器换算速度')
 axes[1,col].axhline(0,ls='--',color='gray',label='速度目标=0')
 axes[1,col].set_ylabel('速度 / m/s');axes[1,col].legend(fontsize=9)
 axes[2,col].plot(x[:,0]-a,x[:,2],label='Roll 估计角')
 axes[2,col].plot(x[:,0]-a,x[:,3],ls='--',alpha=.7,label='Roll 目标')
 axes[2,col].set_ylabel('Roll / °');axes[2,col].set_xlabel('本次启动后 PC 时间 / s');axes[2,col].legend(fontsize=9)
 for ax in axes[:,col]:ax.grid(alpha=.2);ax.set_xlim(0,b-a)
fig.suptitle('2026-09-21 实车：零速度目标下的前后失稳\n未记录 Pitch 目标/PWM、手扶与打滑，曲线不能单独确认反馈方向',fontsize=14)
fig.savefig(out/'balance-traces.png',dpi=160);plt.close(fig)
print('Report data and figure:',out)
print('latest_params',series[-1][-1]);print('table')
for e in summary['episodes']:print(e['id'],round(e['duration_seconds'],2),round(e['speed_min_mps'],3),round(e['speed_max_mps'],3),round(e['pitch_min_deg'],2),round(e['pitch_max_deg'],2))
