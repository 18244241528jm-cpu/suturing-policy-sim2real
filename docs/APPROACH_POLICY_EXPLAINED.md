# The Approach policy: what it is, how well it works, and what we do not know

Written to be read straight through before a supervision meeting. Every number here is
traceable to a file named in the last section; where a number is soft, it says so.

---

## 1. What the model actually does

One subtask, called **Approach**: move the PSM gripper from wherever it is to a *grasp
pose* on the needle. It does not close the jaw and does not lift. Grasping and the rest
of the suturing sequence are separate problems.

**In:** where the gripper is now, and where it should end up.
**Out:** how far to step next.

```
    observation  = concat(current[7], goal[7], goal - current[7])     21 numbers
    achieved_goal = current[7]
    desired_goal  = goal[7]

    each 7-vector = (x, y, z, roll, pitch, yaw, jaw)
    action        = 7 numbers in [-1, 1], multiplied by a fixed step size
    step size     = 1 mm per axis, 3 deg per axis, 0.05 jaw
```

So one forward pass moves the gripper at most 1 mm and 3 degrees. A full approach is
tens to hundreds of these steps. The policy is a *local* controller, not a planner.

**Algorithm:** TD3 with Hindsight Experience Replay and a behaviour-cloning auxiliary
loss, in a project-specific `TD3_HER_BC` class. TD3 for continuous control, HER because
the reward is sparse and the goal changes every episode, BC because the demonstrations
are the only thing that gets it off the ground in a reasonable number of steps.

**Reward:** sparse. Success during training means the measured translation error is
within 1 cm and the orientation within 10 degrees.

> **Anticipated question -- "Why sparse, why not shape it?"**
> A shaped reward for a 6-DOF pose target requires choosing a weighting between
> millimetres and degrees, and that choice silently decides what the policy optimises.
> Sparse plus HER plus demonstrations avoids inventing that exchange rate.

---

## 2. What it was trained on

Demonstrations come from a scripted geometric expert driven in the simulator, not from
human teleoperation. Each episode places the needle, computes the grasp goal from the
needle pose, and records the expert's trajectory. An episode is kept only if the expert
finished within an acceptance threshold.

| | poseclose | wide15 | wide5 |
|---|---|---|---|
| Demonstrations accepted | 50 | **69 / 100** | **99 / 100** |
| Transitions | 10 611 | 11 516 | 15 713 |
| Goal jitter (how far the goal is displaced) | none | **± 15 mm** | **± 5 mm** |
| Needle placement noise, *collection* | 0.3 / 0.2 mm | 3.0 / 3.0 mm | 0.3 / 0.2 mm |
| Needle placement noise, *online rollouts* | 0.3 / 0.2 mm | 3.0 / 3.0 mm | **3.0 / 3.0 mm** |
| Demo acceptance threshold | 1.0 mm | 2.0 mm | 2.0 mm |
| Training steps | 150 000 | 150 000 | 150 000 |
| Wall clock | 3 h 52 | 2 h 00 | 3 h 20 |

**Why jitter the goal at all.** The needle reset in simulation randomises x, y and yaw
but never z, so the goal's absolute height is a constant across the entire original
training set -- the policy never saw z vary. On hardware, calibration and perception
error displace the goal in all three axes. Jitter models that error by moving the *goal*
rather than the needle, which is the honest analogue: on the cart the needle sits still
and it is our estimate of it that is wrong.

> **Anticipated question -- "Why did 31 of 100 demonstrations fail at 15 mm?"**
> Acceptance is strongly z-dependent. Binned by the goal's z displacement, the expert
> succeeded 100 % at +10..+15 mm and only 17.6 % at -15..-10 mm. The likely mechanism is
> the 7 mm hover built into the grasp pose: displace the goal far enough downward and the
> target sits at or below the phantom surface, where the expert cannot reach it. Grasp
> angle showed no such effect (12.59 deg on successes against 12.30 deg on failures), so
> it is z, not the approach angle.

---

## 3. How well it works

### 3.1 The number you should quote

Under a goal-offset sweep in the simulator, at a **3 mm** success criterion, five
episodes per cell:

```
  goal z error |  poseclose (narrow)      |  wide5 (widened)
      (mm)     |  succ   trans    angle   |  succ   trans    angle
      -15      |  0/5    76.2mm    158    |  0/5     9.9mm     62
      -10      |  0/5    20.2mm    102    |  0/5    10.7mm     88
       -8      |  0/5    12.0mm    108    |  0/5    10.2mm     94
       -5      |  0/5     4.9mm    106    |  2/5     6.5mm     19
       -2      |  1/5     4.7mm     89    |  3/5     3.6mm     10
       +0      |  4/5     4.3mm     24    |  4/5     3.3mm     10
       +2      |  4/5     3.5mm     36    |  2/5     5.0mm     12
       +5      |  1/5     8.2mm    131    |  2/5     5.6mm     13
       +8      |  0/5    11.2mm    176    |  1/5     5.3mm     31
      +10      |  0/5    14.7mm    173    |  1/5     7.5mm     29
      +15      |  0/5    96.0mm    124    |  1/5     6.5mm     38
```

Three readings, in order of how much they matter:

**Peak accuracy did not improve.** Both models manage 4/5 at zero goal error. Widening
bought nothing at the sweet spot, and anyone hoping for a more accurate policy should
read this row and stop.

**The usable band widened about threefold.** Cells with any success at all span 7 mm for
poseclose (-2 to +5) and 20 mm for wide5 (-5 to +15).

**The cliff disappeared, and this is the result that matters.** Outside its band
poseclose does not merely degrade -- it flips and departs, up to 96 mm of translation
error and 176 degrees of orientation error, which is essentially the needle-symmetry
flip. Across the entire +/-15 mm sweep wide5 never exceeds 10.7 mm.

> **Anticipated question -- "So is the new model better?"**
> It is more *robust*, not more *accurate*. On hardware the goal will be wrong by some
> millimetres, because calibration error and perception error add. With poseclose, being
> a few millimetres off sends the arm somewhere unrelated. With wide5, being a few
> millimetres off leaves the arm a few millimetres off. That is the difference between a
> failure you can measure and one you have to abort.

### 3.2 Training-time behaviour

```
    wide15   peak 0.52 at  27 700 steps  ->  0.19 at 150 000   monotone collapse
    wide5    peak 0.94 at 106 400 steps  ->  0.63 at 150 000   late peak, slow decline
```

Both decline after their peak, which is why the final checkpoint is not the one to
deploy; a separate `best_model.zip` is kept whenever the rolling success rate improves.
wide5's peak is nearly double wide15's and arrives four times later, which is what a run
that is actually converging looks like rather than one that finds a degenerate policy
early.

> **Anticipated question -- "Why does success fall after the peak?"**
> Not established. Candidates are TD3 value overestimation, the BC term weakening
> relative to the RL term as training proceeds, and the online rollouts drifting from the
> demonstration distribution. Distinguishing them needs an ablation nobody has run. The
> practical consequence is handled -- deploy the best checkpoint, not the last -- but the
> cause is open.

> **Anticipated question -- "Is 0.94 the same 0.94 as the capability card's 4/5?"**
> No, and this trips people up. The training figure is a rolling average over the last 50
> *training* episodes at a **1 cm** criterion with randomised resets. The capability card
> is a held-out evaluation at a **3 mm** criterion with fixed resets. The 3 mm number is
> the one to quote externally.

---

## 4. The measurement problem, which is the real story

The first evaluation of the widened model returned 0/5 in all 20 cells with errors above
one metre. That reads as a total failure of the retrain. It was not.

Running the **known-good** June baseline through the same evaluation produced 1521 mm as
well. An instrument that scores a working model and a broken model identically is
measuring itself.

Two independent defects, both in the evaluation rather than the policy:

**The policy was driven from the wrong feedback.** The evaluation variant integrated its
own commands and fed the result back as the observed pose -- steering by where the wheel
was told to go instead of where the car went. The policy was trained against *measured*
pose. Error accumulated without bound; hence the metre-scale numbers.

**The reference goal was not a pose.** The sweep centres itself on the mean of the
checkpoint's training goals, and computed that mean by averaging the 7-vectors
element-wise -- including indices 3, 4, 5, which are Euler angles. Averaging Euler angles
is not averaging rotations. The resulting "mean orientation" sat **60.1 degrees** away
from the real grasp orientation and need not correspond to any reachable grasp at all.
The policy was servoing towards a target that does not exist, which is why every cell
reported a near-constant ~173 degrees of final orientation error regardless of offset.

Fixing the driving contract alone dropped translation error from 1521 mm to 8-31 mm but
left the orientation nonsense; only offsetting around a **live** goal produced the valid
sweep in section 3.1. Three attempts, and the first two were measuring the instrument.

> **Anticipated question -- "How do you know the third measurement is valid?"**
> Its zero-offset cell reproduces the independently known behaviour of the model. The
> July checkpoint scores 5/5 at 2.53 mm under a plain measured-contract evaluation, and
> 4/5 at 4.31 mm as the sweep's zero cell -- consistent within five-episode noise. The
> first two attempts failed that same check, which is what condemned them.

> **Anticipated question -- "Does this invalidate earlier results?"**
> Yes, the policy numbers. The previously reported usable band of z in [-2, +10] mm was
> measured with both defects present and should not be quoted. The geometric servo's
> numbers are unaffected: it wraps angles explicitly and is never driven through the
> command-integrated path.

---

## 5. What we do not know

**The retrain is not a clean single-variable comparison.** wide15 changed two widths at
once -- goal jitter 0 to 15 mm *and* needle placement noise 0.3/0.2 to 3.0/3.0 mm -- so
its collapse cannot be attributed. wide5 was meant to fix that and only half did: the
goal jitter came down to 5 mm and the demonstrations were re-collected at 0.3/0.2 mm, but
the training script has no pass-through for needle noise, so the **online rollouts still
ran at 3.0/3.0 mm**. wide5's demonstrations and its own rollouts therefore disagree.
The comparison is cleaner than wide15's and still not clean.

**Peak success is 4/5, not 5/5.** Even at zero goal error one episode in five fails at
the 3 mm criterion. Nothing here explains that fifth episode.

**Everything above is simulation.** The policy has never run on hardware. There is now a
node that can run it, but no result.

**The 2 mm figure recurs and is not fully explained.** Three independent measurements
land near 2 mm -- FoundationPose fails above roughly 2 mm of depth shape error, the
policy's usable z band is about +/-2 mm, and the earlier end-to-end budget was 2 mm.
Whether that is one shared cause or a coincidence of three unrelated limits is unknown.

---

## 6. If you are asked what to do next

1. **Fix the training script's pass-through** so needle noise can be set for the online
   rollouts, then rerun wide5 as a genuine single-variable comparison. Until then the
   attribution of wide15's collapse is an inference, not a measurement.
2. **Sweep axes other than z.** The capability card covers z only. x, y and yaw are
   unmeasured under the corrected lens.
3. **Deploy the geometric servo first.** It beat the policy on every axis that was
   measured, carries no weights, and therefore has no sim-to-real transfer to go wrong.
   The policy is the second experiment, not the first.

---

## 7. Where the numbers come from

| Claim | Source |
|---|---|
| Capability cards | `~/sweep_xoff_wide5/`, `~/sweep_xoff_poseclose/` |
| Failed and invalid sweeps | `~/sweep_wide/` (contract defect), `~/sweep_o2_poseclose/` (Euler-mean defect) |
| Baseline cross-checks | `~/sweep_ctrl_oldmodel/`, `~/sweep_ctrl_poseclose/`, `~/ctrl_X_poseclose.json` |
| Training curves and configs | `wide5/training_run.json`, `wide15/training_run.json`, and the `train.log` beside each |
| Demonstration statistics | `~/planB_20260826/demos_wide5/collection_report.json` |
| Checkpoints | `wide5/checkpoints/best_model.zip` (106 431 steps, 0.94) |
| Euler-mean defect | `Model_evaluation.py`, `load_checkpoint_goal_offset_sweep_goals` |
| Observation contract | `Approach_env.py` around the reset observation, and `gym_manager.normalize_observation` |
