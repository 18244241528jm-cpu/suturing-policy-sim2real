# Architecture and contracts

## 1. Deployment data flow

```mermaid
flowchart LR
  RGB[ECM RGB] --> DA[Metric depth]
  MASK[Needle mask] --> FP[FoundationPose register]
  DA --> FP
  FP --> GATE[Support-plane and R_rest gate]
  GATE --> GOAL[Frozen needle grasp goal]
  DVRK[dVRK measured_cp] --> HE[Hand-eye transform]
  HE --> PSM[PSM pose in camera frame]
  GOAL --> CTRL[Staged SE(3) controller]
  PSM --> CTRL
  CTRL --> REACH[Reach]
  REACH --> CLOSE[Physical close/lift: not validated]
```

The first demonstration freezes the needle goal after a checked first-frame
registration. This is valid only while the ECM and needle remain stationary.

## 2. Coordinate convention

Every transform follows:

```text
T_A_from_B maps coordinates expressed in frame B into frame A.
p_A = T_A_from_B @ p_B
```

Needle FoundationPose is expressed in the left-camera frame. PSM `measured_cp`
is expressed in its robot base frame. Hand-eye calibration connects them.
Never compare or subtract poses before both are in the same frame.

## 3. Perception contract

FoundationPose receives RGB, metric depth, a binary object mask and a CAD mesh.
The mask tells it *where* the object is; it does not resolve a symmetric 6D
orientation. `register` generates many hypotheses and ranks them. `track`
optimizes locally from the previous estimate.

The first-frame gate adds information independent of FP score:

- distance to the support plane;
- consistency with the needle resting orientation `R_rest`;
- rejection of physically impossible flipped branches.

Repeated registration or adjacent-frame agreement is not a correctness test:
a symmetric wrong branch can be stable across frames.

## 4. Simulation privilege boundary

AMBF semantic masks are renderer object IDs. They are perfect labels, not a
real segmentation algorithm. AMBF GT depth and body pose are also privileged.

The deployment-proxy path replaces inputs layer by layer:

```text
GT RGB/depth/pose/mask
  -> rendered RGB + learned depth
  -> perturbed manual-mask proxy
  -> FoundationPose + physical gate
  -> frozen goal
  -> biased kinematics/hand-eye proxy
  -> controller
```

Passing a downstream controller with semantic masks does not prove an
automatic real-camera pipeline.

## 5. Control contract

The D2 controller computes a relative SE(3) correction, uses geodesic SO(3)
error, gain scheduling and bounded translation/rotation steps. Large rotation
is reduced before full translation. Reach, scripted attachment and physical
close/lift are reported as separate outcomes.

## 6. RL contract

The released Approach code uses measured observations, a single desired goal,
pose-close success and the configured needle reset range. Demonstration,
training termination and evaluation must use the same goal semantics. Historical
25-candidate termination results are not directly comparable.

