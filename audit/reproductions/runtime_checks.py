#!/usr/bin/env python3
"""Reproduce audit observations without modifying the implementation."""
import argparse
import json
import math
from pathlib import Path
import sys
from unittest.mock import patch

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--repo', type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument('--output', type=Path, default=Path('/tmp/multiroi-audit-runtime-results.json'))
args = parser.parse_args()
sys.path.insert(0, str(args.repo.resolve()))
import cv2
import numpy as np
from scipy.interpolate import LSQUnivariateSpline
from test_multi_roi import MultiROIDetector
from mr_vs import MultiROIVS, MRVSParams
from temporal_filter import TemporalNavigationFilter, TemporalFilterParams
from lookahead_corridor_map import LookaheadCorridorMap, LookaheadParams
from run_mr_navigation import process_image
import run_mr_navigation


def rows(lean=0.0):
    image = np.full((480,640,3), (60,65,120), np.uint8)
    dx = round(480*math.tan(math.radians(lean)))
    for xb in (250,390):
        pts = np.array([[xb-5,479],[xb+5,479],[xb+dx+5,0],[xb+dx-5,0]],np.int32)
        cv2.fillPoly(image,[pts],(30,200,30))
    return image


def process(image, use_map=False, **det_params):
    det = MultiROIDetector(morph='struct', **det_params)
    return process_image(image, det, MultiROIVS(MRVSParams()), draw=False,
        t_filter=TemporalNavigationFilter(TemporalFilterParams(n_strips=det.n)),
        lookahead_map=LookaheadCorridorMap() if use_map else None,
        dt=.05, last_w=0.)


def F01():
    values=[]
    for use_map in (False,True):
        out=process(rows(8),use_map)
        values.append({'lookahead':use_map,'theta_deg':out['info']['filt_err_theta_deg'],
                       'w_raw':out['info']['w_raw']})
    return {'reproduced':values[0]['theta_deg']*values[1]['theta_deg']<0,'observed':values}


def F02():
    a,b=process(rows(),False),process(rows(),True)
    result={'without_map_px':a['info']['filt_err_x'],'with_map_px':b['info']['filt_err_x'],
            'crop_offset':list(a['res']['crop_offset'])}
    return {'reproduced':abs((result['with_map_px']-result['without_map_px'])+12)<.01,'observed':result}


def F03():
    det=MultiROIDetector(n_strips=8)
    memory=LookaheadCorridorMap(LookaheadParams(n_bins=10))
    for frame in range(2):
        res=det.detect(rows())
        try:memory.update(res['strip_profile'],res,(480,640))
        except IndexError as exc:
            return {'reproduced':frame==1,'observed':{'frame_index':frame,'error':str(exc)}}
    return {'reproduced':False,'observed':'No IndexError'}


def F04():
    image=np.full((480,640,3),(60,65,120),np.uint8)
    det=MultiROIDetector(morph='struct');vs=MultiROIVS()
    tf=TemporalNavigationFilter()
    for _ in range(40):out=process_image(image,det,vs,draw=False,t_filter=tf,dt=.05,last_w=0.)
    result={'two_sided':out['res']['n_two_sided'],'accepted_points':out['res']['n_q_accepted'],
            'has_line':out['raw_dict']['has_line'],'v_mps':out['v'],
            'status':out['info']['status'],'confidence':out['info']['confidence']}
    return {'reproduced':result['two_sided']==0 and result['has_line'] and result['v_mps']>0,'observed':result}


def F05():
    out=process(rows())
    result={'info_has_line_present':'has_line' in out['info'],
            'filtered_has_line':out['filt_out']['has_line'],
            'two_sided':out['res']['n_two_sided']}
    return {'reproduced':not result['info_has_line_present'] and result['filtered_has_line'],'observed':result,
            'scope':'Reproduces the missing interface field; no Gazebo run performed'}


def F06():
    values=[]
    for elapsed in (.001,.4):
        vs=MultiROIVS(MRVSParams(ki=0.))
        with patch('mr_vs.time.perf_counter',side_effect=[1.,1.+elapsed]):
            vs.compute_control(np.array([100.,0.,0.]),confidence=1.)
            _,w,_=vs.compute_control(np.array([100.,0.,0.]),confidence=1.)
        values.append({'elapsed_host_seconds':elapsed,'w_radps':w})
    return {'reproduced':abs(values[0]['w_radps']-values[1]['w_radps'])>.05,'observed':values,
            'scope':'Deterministic clock injection into the existing wall-clock controller path'}


def raw(x=320.,n=9,has_line=True):
    return {'raw_bottom_x':x,'raw_theta':0.,'raw_width':140.,'raw_X':x-320.,
            'has_line':has_line,'n_two_sided':n,'n_q_accepted':n,'n_q_rejected':0,'weed_pressure':0.}


def F07():
    values=[]
    for strips,n in [(10,6),(5,5)]:
        tf=TemporalNavigationFilter(TemporalFilterParams(n_strips=strips))
        tf.update(raw(n=n),dt=.05)
        for _ in range(200):out=tf.update(raw(420.,n),dt=.05)
        values.append({'configured_strips':strips,'two_sided':n,'filtered_bottom_x':out['filt_bottom_x'],
                       'status':out['status'],'pending_count':out['pending_count']})
    evidence_counts = {str(ignored): MultiROIDetector(ignore_initial=ignored).detect(rows())['n_two_sided']
                       for ignored in (0, 3, 4)}
    return {'reproduced':all(v['filtered_bottom_x']==320. and v['status']=='pending' for v in values),
            'observed':{'filter_runs': values, 'detector_two_sided_by_ignore_initial': evidence_counts},
            'scope':'Filter hold and detector evidence limit reproduced; no closed-loop trajectory measured'}


def F09():
    tf=TemporalNavigationFilter();tf.update(raw(),dt=.05)
    states=[]
    for k in range(7):
        out=tf.update(raw(420.),dt=.05)
        states.append(out['status'])
        if k<6:tf.update(raw(322.,0),dt=.05)
    tf=TemporalNavigationFilter();tf.update(raw(),dt=.05)
    for _ in range(6):tf.update(raw(420.),dt=.05)
    for _ in range(100):tf.update(raw(n=0,has_line=False),dt=.05)
    out=tf.update(raw(420.),dt=.05)
    result={'interleaved_statuses':states,'after_100_missing_frames_then_one_observation':out['status'],
            'filtered_bottom_x_after_gap':out['filt_bottom_x']}
    return {'reproduced':states[-1]=='pending_accepted' and out['status']=='pending_accepted','observed':result}


def F08():
    import ast
    from types import SimpleNamespace
    source = args.repo / 'tests/sim_ros2/src/mrsim/mrsim/nav_node.py'
    tree = ast.parse(source.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef)
               and node.name == 'MultiROINavNode')
    callback = next(node for node in cls.body if isinstance(node, ast.FunctionDef)
                    and node.name == '_on_image')
    namespace = {'Image': object}
    exec(compile(ast.Module(body=[callback], type_ignores=[]), str(source), 'exec'), namespace)
    publications, stops, errors = [], [], []
    def fail(_image):
        raise RuntimeError('Injected perception failure')
    node = SimpleNamespace(last_img_t=99., frame_idx=1, t0=0., max_seconds=1.,
        idle=False, lane_end_x=10., odom_x=100.,
        _last_cmd=SimpleNamespace(linear=SimpleNamespace(x=.2)),
        pipeline=SimpleNamespace(process=fail, name='fault injection'),
        cmd_pub=SimpleNamespace(publish=publications.append),
        _stop_robot=stops.append, _decode_bgr=lambda _msg: rows(),
        get_logger=lambda: SimpleNamespace(error=errors.append))
    for stamp in (100, 101, 102):
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp,nanosec=0)))
        namespace['_on_image'](node, msg)
    result = {'failed_callbacks': len(errors), 'new_command_publications': len(publications),
              'stop_calls': len(stops), 'last_command_v_mps': node._last_cmd.linear.x,
              'max_seconds': node.max_seconds, 'last_sim_seconds': 102,
              'odom_x': node.odom_x, 'lane_end_x': node.lane_end_x}
    return {'reproduced': len(errors)==3 and not publications and not stops,
            'observed': result,
            'scope': 'Executes the unchanged callback body with stub ROS objects and injected pipeline failures. No Gazebo plugin or live ROS test.'}


def F10():
    ys=np.arange(60,380,40.);xs=np.full_like(ys,300.);xs[4]=380.
    captured={}
    def trace(frame,event,arg):
        if frame.f_code.co_name=='_fit_nav_curve' and event=='return':
            captured['kept_points']=len(frame.f_locals.get('keep',[]))
        return trace
    previous=sys.gettrace();sys.settrace(trace)
    try:samples,_=MultiROIDetector()._fit_nav_curve(list(zip(xs,ys)),462,env_cap=1e9)
    finally:sys.settrace(previous)
    n_int=max(0,min(3,(len(ys)-3)//2));knots=np.linspace(ys[0],ys[-1],n_int+2)[1:-1]
    full=LSQUnivariateSpline(ys,xs,knots,k=3)
    yy=np.array([p[1] for p in samples]);inside=(yy>=ys[0])&(yy<=ys[-1])
    error=float(np.max(np.abs(np.array([p[0] for p in samples])[inside]-full(yy[inside]))))
    result={'maximum_difference_from_all_points_spline':error,'input_points':len(ys),**captured}
    return {'reproduced':error<1e-9,'observed':result,
            'scope':'Reports whether the proposed sample actually removed any point as well as spline agreement'}


def A01():
    points=[(300,400),(300,300),(300,200),(300,100)]
    values=[]
    for pts in (points,list(reversed(points))):
        line,_,_=MultiROIDetector._fit_line_tls(pts)
        values.append({'line':list(line),'x_at_y_250':(250-line[1])/line[0]})
    det=MultiROIDetector(morph='struct',nav_curve=False)
    out=process_image(rows(),det,MultiROIVS(),draw=False,dt=.05)
    result={'expected_x':300,'point_order_results':values,'healthy_straight_corridor':{
        'two_sided':out['res']['n_two_sided'],'lateral_error_px':out['info']['err_x'],
        'v_mps':out['v'],'w_radps':out['w']}}
    return {'reproduced':abs(values[0]['x_at_y_250']-300)>500 and abs(values[1]['x_at_y_250']-300)<.01,
            'observed':result}


def A02():
    values=[]
    for dt in (.05,.01,.005):
        vs=MultiROIVS(MRVSParams(ki=0.))
        for _ in range(100):v,w,info=vs.compute_control(np.array([100.,0.,0.]),dt=dt,confidence=1.,smooth=True)
        values.append({'dt_seconds':dt,'command_after_100_updates':w,'raw_command':info['w_raw']})
    return {'reproduced':values[0]['command_after_100_updates']!=0 and all(v['command_after_100_updates']==0 for v in values[1:]),
            'observed':values}


def P06():
    import argparse
    captured = {}
    original = argparse.ArgumentParser.parse_args
    class ParserCaptured(Exception):
        pass
    def capture(parser, *unused_args, **unused_kwargs):
        captured.update(vars(original(parser, [])))
        raise ParserCaptured
    with patch.object(argparse.ArgumentParser, 'parse_args', capture):
        try:
            run_mr_navigation.main()
        except ParserCaptured:
            pass
    defaults = MRVSParams()
    result = {'cli_lambdax': captured['lambdax'], 'library_lambdax': defaults.lambda_x,
              'cli_wmax': captured['wmax'], 'library_wmax': defaults.w_max}
    return {'reproduced': result['cli_lambdax'] == 5 * result['library_lambdax'],
            'observed': result, 'scope': 'Configuration divergence; stability consequences not measured'}


def profile(center=200., valid_count=10):
    return [dict(mu=i+1, y_center=480-(i+.5)*36, center_x=center,
                 width=120. if i<valid_count else None,
                 left_x=center-60 if i<valid_count else None,
                 right_x=center+60 if i<valid_count else None,
                 two_sided=i<valid_count, accepted_nav=i<valid_count)
            for i in range(10)]


def P07():
    memory=LookaheadCorridorMap()
    observed=[]
    for _ in range(10):
        out=memory.update(profile(valid_count=5),{'median_width':120.},(480,640))
        pred=memory.get_prediction((480,640))[4]
        observed.append({'status':out['per_strip_status'][4],
                         'predicted_center':pred.center_x,'confidence':pred.confidence})
    return {'reproduced':abs(observed[-1]['predicted_center']-200)>40.,
            'observed':{'true_center':200.,'frames':observed}}


def P08():
    memory=LookaheadCorridorMap(LookaheadParams(shift_px=0.))
    for _ in range(5):memory.update(profile(),{'median_width':120.},(480,640))
    prof=profile();prof[0].update(two_sided=False,width=None,left_x=None,right_x=None)
    out=memory.update(prof,{'median_width':120.},(480,640))
    missing={'map_status':out['map_status'],'bottom_missing':out['bottom_missing'],
             'bottom_consistent':out['bottom_consistent'],'proxy_strip_status':out['per_strip_status'][1]}
    memory=LookaheadCorridorMap(LookaheadParams(shift_px=0.,ignore_initial=3))
    for _ in range(5):memory.update(profile(),{'median_width':120.},(480,640))
    prof=profile()
    for p in prof[3:]:p.update(center_x=260.,left_x=200.,right_x=320.)
    out=memory.update(prof,{'median_width':120.},(480,640))
    support={'effective_bottom_index':3,'effective_bottom_center':260.,
             'ignored_strip_0_center':200.,'support_new':out['spatial_support_new'],
             'upper_strips_matching_effective_bottom':7,'map_status':out['map_status']}
    return {'reproduced':not missing['bottom_consistent'] and missing['proxy_strip_status']=='map_accept'
                         and support['support_new']==0,
            'observed':{'missing_bottom':missing,'ignored_bottom_support':support}}


def P09():
    det=MultiROIDetector();res=det.detect(rows());memory=LookaheadCorridorMap()
    detector_step=res['strip_profile'][0]['y_center']-res['strip_profile'][1]['y_center']
    map_step=480*memory.params.vertical_coverage/memory.params.n_bins
    result={'detector_step_px':detector_step,'map_step_px':map_step,
            'crop_offset':list(res['crop_offset']),
            'detector_coverage':res['vertical_coverage']}
    return {'reproduced':detector_step!=map_step,'observed':result,
            'scope':'Grid mismatch measured; prediction and overlay impact depends on scene'}


def P10():
    try:MultiROIDetector(n_strips=1).detect(rows())
    except UnboundLocalError as exc:
        return {'reproduced':True,'observed':{'error':str(exc)}}
    return {'reproduced':False,'observed':'No UnboundLocalError'}


def json_scalar(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'Unsupported result type: {type(value).__name__}')


results={}
for name in ['F01','F02','F03','F04','F05','F06','F07','F08','F09','F10','A01','A02',
             'P06','P07','P08','P09','P10']:
    try:results[name]=globals()[name]()
    except Exception as exc:results[name]={'reproduced':False,'unexpected_error':type(exc).__name__+': '+str(exc)}
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps({'repository':str(args.repo.resolve()),'results':results},indent=2,default=json_scalar)+'\n')
print(json.dumps(results,indent=2,default=json_scalar))
print('Saved',args.output)
sys.exit(0 if all(case['reproduced'] for case in results.values()) else 1)
