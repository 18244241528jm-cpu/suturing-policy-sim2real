import os
_src = open(os.path.expanduser('~/r6off/exp1/exp1.py')).read().split('# ---------------- sensitivity grid')[0]
exec(compile(_src, 'exp1_head', 'exec'))
from surgicai_rl_deploy.contract import R6_START_OFFSET_TOOL_MEAN
R_ = {}
rows = {}
for t in TR:
    ts = json.load(open(f'{IN}/trials/{t}/trial_state.json')) if os.path.exists(f'{IN}/trials/{t}/trial_state.json') else {}
    h = ts.get('hover') or {}
    c = h.get('corrections') or []
    if not c: continue
    first = c[0]
    rows[t] = {'first_err_norm_mm': first.get('err_norm'), 'first_lateral_mm': first.get('lateral_mm'), 'first_height_mm': first.get('height_mm'), 'n_corrections': h.get('n_corrections'),
               'applied_mm': h.get('applied_mm'), 'keys0': list(first.keys())[:8]}
    print('d)', t, 'first err_norm %s lateral %s height %s | n_corr %s applied %s' % tuple((round(x, 2) if isinstance(x, float) else x) for x in (first.get('err_norm'), first.get('lateral_mm'), first.get('height_mm'), h.get('n_corrections'), [round(a, 1) for a in h.get('applied_mm', [])])))
v = [r['first_err_norm_mm'] for r in rows.values() if r['first_err_norm_mm'] is not None]
print('d) median first err_norm %.2f mm [%.2f, %.2f] n=%d' % (np.median(v), min(v), max(v), len(v)))
R_['d_day_open_loop_hover_error'] = rows
Rel = Rot.from_rotvec(R6_START_TO_GOAL_ROTVEC_MEAN).as_matrix(); OFF = np.asarray(R6_START_OFFSET_TOOL_MEAN) / 100.0
mins = {}
for t in TR:
    a = A[t]; goal, _ = goal_pose(a, 'GO_rel', 0.007); Rs = goal.R @ Rel.T; start = Pose(goal.p - Rs @ OFF, Rs, 0.0)
    r = run_policy(start, goal, 0.76); H = r['hist']
    d = []
    for h_ in H:
        p, R = endpoint_robot(r['Xb'], h_); d.append((np.linalg.norm(p - goal.p) * 1000.0, np.degrees(Rot.from_matrix(R.T @ goal.R).magnitude())))
    d = np.array(d); k = int(np.argmin(d[:, 0])); mins[t] = {'min_d_mm': float(d[k, 0]), 'step': k, 'rot_deg_at_min': float(d[k, 1]), 'first_success': r['first']}
print('e) staged GO_rel: min distance along trajectory median %.1f mm [%.1f, %.1f]; step median %d; rot at min median %.1f deg' % (np.median([m['min_d_mm'] for m in mins.values()]), min(m['min_d_mm'] for m in mins.values()), max(m['min_d_mm'] for m in mins.values()), np.median([m['step'] for m in mins.values()]), np.median([m['rot_deg_at_min'] for m in mins.values()])))
R_['e_min_along_trajectory'] = mins
json.dump(R_, open(OUT + '/exp1c_results.json', 'w'), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
