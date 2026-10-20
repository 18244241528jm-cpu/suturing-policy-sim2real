import os
_src = open(os.path.expanduser('~/r6off/exp1/exp1.py')).read().split('# ---------------- sensitivity grid')[0]
exec(compile(_src, 'exp1_head', 'exec'))
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from surgicai_rl_deploy.contract import R6_START_OFFSET_TOOL_MEAN
Rel = Rot.from_rotvec(R6_START_TO_GOAL_ROTVEC_MEAN).as_matrix(); OFF = np.asarray(R6_START_OFFSET_TOOL_MEAN) / 100.0
def curve(a, orient, staged):
    goal, st = goal_pose(a, orient, 0.007)
    if goal is None: return None
    if staged: Rs = goal.R @ Rel.T; st = Pose(goal.p - Rs @ OFF, Rs, 0.0)
    r = run_policy(st, goal, 0.76); d = []
    for h in r['hist']:
        p, R = endpoint_robot(r['Xb'], h); d.append(np.linalg.norm(p - goal.p) * 1000.0)
    return np.array(d), r['first']
fig, ax = plt.subplots(1, 3, figsize=(15, 4.6), sharey=True)
for k, (orient, staged, ttl) in enumerate((('GO_day', False, 'day start, goal = day hover orientation (primary)'), ('GO_rel', False, 'day start, goal = trained-relative orientation'), ('GO_rel', True, 'start placed INSIDE the training support'))):
    for t in TR:
        c = curve(A[t], orient, staged)
        if c is None: continue
        d, f = c; ax[k].plot(d, lw=0.9, alpha=0.7); 
        if f: ax[k].plot(f, d[f], 'k.', ms=4)
    ax[k].axhline(10, c='gray', ls='--', lw=0.8); ax[k].axhline(5, c='gray', ls=':', lw=0.8); ax[k].set_title(ttl, fontsize=9); ax[k].set_xlabel('policy step'); ax[k].set_ylim(0, 80)
ax[0].set_ylabel('distance to the policy\'s own goal (mm)')
fig.text(0.01, 0.005, 'R6 offline 2026-10-16 | perfect execution, needle static, standoff 7 mm, jaw 0.76 | black dot = first step meeting 1 cm / 10 deg | dashed 10 mm, dotted 5 mm | staged: all 14 curves identical (rebase)', fontsize=7)
fig.tight_layout(rect=[0, 0.03, 1, 1]); fig.savefig(os.path.expanduser('~/r6off/figs/exp1_distance_vs_step.png'), dpi=120)
R3 = json.load(open(os.path.expanduser('~/r6off/exp3/exp3_results.json')))['per_placement']
fig, ax = plt.subplots(figsize=(8, 4.2)); names = list(R3)
ax.semilogy(range(len(names)), [R3[n]['pad_sd_mm']['FS'] for n in names], 'o', label='FS'); ax.semilogy(range(len(names)), [R3[n]['pad_sd_mm']['SGBM'] for n in names], 's', label='SGBM (untuned)')
ax.axhline(1.0, c='gray', ls='--', lw=0.8); ax.set_xticks(range(len(names))); ax.set_xticklabels([n.replace('_P0', '').replace('_C', '').replace('_S', '') for n in names], rotation=60, fontsize=7)
ax.set_ylabel('pad quadratic-surface residual sd (mm)'); ax.legend(); ax.set_title('Pad-surface noise, snap0 of 13 placements')
fig.text(0.01, 0.005, 'R6 offline 2026-10-16 | pad = pink annulus 10-25 px from the clicked polyline | dashed = ~needle height 1 mm', fontsize=7)
fig.tight_layout(rect=[0, 0.03, 1, 1]); fig.savefig(os.path.expanduser('~/r6off/figs/exp3_pad_noise.png'), dpi=130); print('ok')
