# Known limitations and negative results

These are boundaries, not hidden TODOs.

## Validated scope

- Fixed ECM and stationary needle.
- First-frame needle registration followed by a frozen goal.
- Learned DA depth validated in the AMBF image domain.
- Manual-mask-equivalent proxy, not real automatic segmentation.
- PSM state from kinematics plus hand-eye proxy.
- D2 staged SE(3) Reach, not physical grasp retention.

## Not established

- Metric DA accuracy on real endoscopic RGB.
- Automatic real needle mask.
- Real hand-eye held-out accuracy.
- Pure visual PSM 6D tracking suitable for control.
- Full live DA/FP freshness with a moving ECM.
- Physical close, lift and hold.
- Place, Insert, Regrasp and Pullout under the same deployment contract.

## Measured negative results

1. Pure PSM FoundationPose tracking remained at or near zero strict pass after
   rigid-link, composite-jaw, multiview and texture experiments.
2. Current 4 mm AMBF stereo had needle depth p95 9.682 mm versus 1.060 mm for
   the new DA checkpoint; robust fusion worsened DA and reduced coverage.
3. At a 5 mm/5 degree proxy error, MHT-EKF improved only 1.43 percentage points
   over fixed fusion and its nominal 95% covariance covered 68.07%.
4. Near-normal camera geometry increased needle pixels but worsened DA needle
   p95 from 1.696 mm to 33.489 mm.
5. Adjacent-frame registration consistency missed a stable approximately
   180-degree wrong branch; consistency is not correctness.

Sources: the dated reports in `docs/evidence/`.

## Interpretation rule

Fusion acts after sensing and candidate generation. It cannot repair an
out-of-distribution depth map or create a correct FP pose that is absent from
the candidate set. Validate each upstream layer before evaluating downstream
control.

