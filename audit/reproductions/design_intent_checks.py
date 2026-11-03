import argparse, json, math, sys
from pathlib import Path
import numpy as np
import cv2
parser = argparse.ArgumentParser(description='Record local sequences for the anti-jitter design review.')
parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--output', type=Path, default=Path('/tmp/multiroi-design-intent-results.json'))
args = parser.parse_args()
sys.path.insert(0, str(args.repo.resolve()))
from test_multi_roi import MultiROIDetector
from run_mr_navigation import process_image
from temporal_filter import TemporalNavigationFilter
from lookahead_corridor_map import LookaheadCorridorMap,LookaheadParams
from mr_vs import MultiROIVS,MRVSParams

def raw(x=320.,theta=0.,n=9):
 return dict(raw_bottom_x=x,raw_X=x-320.,raw_theta=math.radians(theta),raw_width=140.,has_line=True,n_two_sided=n,n_q_accepted=n,n_q_rejected=0,weed_pressure=0.)

def profile():
 return [dict(mu=i+1,y_center=480-(i+.5)*36,center_x=200.,width=120.,left_x=140.,right_x=260.,two_sided=True,accepted_nav=True) for i in range(10)]

out={}
f=TemporalNavigationFilter();f.update(raw(),dt=.05)
xs=[];ths=[];status=[]
for r in [raw(420.,20.,3)]*3+[raw()]*10:
 o=f.update(r,dt=.05);xs.append(o['filt_bottom_x']);ths.append(o['filt_theta_deg']);status.append(o['status'])
out['short_false_excursion']={'expected':'Hold corridor through three false measurements; recover when correct view returns','max_center_deviation_px':max(abs(x-320) for x in xs),'max_heading_deviation_deg':max(abs(x) for x in ths),'statuses':status}
f=TemporalNavigationFilter();f.update(raw(),dt=.05)
for _ in range(100):o=f.update(raw(420.,20.,3),dt=.05)
out['persistent_low_support_false_track']={'expected':'Persistent but poorly supported false track must not replace trusted corridor','center_px':o['filt_bottom_x'],'heading_deg':o['filt_theta_deg'],'status':o['status'],'pending_frames':o['pending_count'],'interpretation':'This same hold was previously used to demonstrate lockout. It is correct when the input is false; the input sequence alone does not establish which corridor is true.'}
f=TemporalNavigationFilter();f.update(raw(),dt=.05)
for k in range(1,101):o=f.update(raw(320.+.5*k,.1*k,9),dt=.05)
out['gradual_supported_change']={'expected':'Track small per-frame lateral and heading changes','measured_center_px':370.,'filtered_center_px':o['filt_bottom_x'],'measured_heading_deg':10.,'filtered_heading_deg':o['filt_theta_deg'],'status':o['status']}
m=LookaheadCorridorMap(LookaheadParams())
for _ in range(15):m.update(profile(),{'median_width':120.},(480,640))
vals=[]
for _ in range(12):
 p=profile();p[0].update(center_x=300.,left_x=240.,right_x=360.)
 o=m.update(p,{'median_width':120.},(480,640));vals.append({k:o[k] for k in ['map_status','raw_bottom_x','corrected_bottom_x','spatial_support_old','spatial_support_new']})
out['false_local_shift_with_consistent_lookahead']={'expected':'Lookahead preserves established corridor when bottom shifts but future supports old corridor','observed':vals}
vals=[]
for dt in (.05,.02,.01):
 vs=MultiROIVS(MRVSParams(ki=0.))
 for _ in range(100):v,w,info=vs.compute_control(np.array([100.,0.,0.]),dt=dt,confidence=1.,smooth=True)
 vals.append({'dt_seconds':dt,'raw_w_radps':info['w_raw'],'output_w_radps':w,'v_mps':v})
out['sustained_steering_by_update_interval']={'expected':'A sustained substantial command should eventually pass through smoothing at every supported update rate','observed':vals}
# Exercise the complete local pipeline, including detector fallback.
image=np.full((480,640,3),(60,65,120),np.uint8)
for x in (210,350):cv2.rectangle(image,(x-5,0),(x+5,479),(30,200,30),-1)
blank=np.full_like(image,(60,65,120))
values=[]
for use_map in (False,True):
 det=MultiROIDetector(morph='struct');vf=MultiROIVS();tf=TemporalNavigationFilter()
 memory=LookaheadCorridorMap() if use_map else None
 for _ in range(15):o=process_image(image,det,vf,draw=False,t_filter=tf,lookahead_map=memory,dt=.05,last_w=0.)
 initial=float(o['filt_out']['filt_bottom_x'])
 for _ in range(20):o=process_image(blank,det,vf,draw=False,t_filter=tf,lookahead_map=memory,dt=.05,last_w=0.)
 values.append({'lookahead':use_map,'before_occlusion_bottom_x':initial,
                'after_20_blank_frames_bottom_x':float(o['filt_out']['filt_bottom_x']),
                'two_sided':o['res']['n_two_sided'],'accepted_points':o['res']['n_q_accepted'],
                'has_line':o['raw_dict']['has_line'],'v_mps':o['v'],
                'status':o['info']['status'],'scope':'Local pipeline, repeated stationary image followed by blank images; no closed-loop motion.'})
out['temporary_full_occlusion_after_established_offcentre_track']={'expected':'Retain or predict from the established corridor rather than follow an unobserved image-centre fallback','observed':values}
args.output.write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
