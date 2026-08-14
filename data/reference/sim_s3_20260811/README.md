# Frozen SIM-S3 reference bank

This compact bank is the minimum upstream evidence needed to replay SIM-S4
without rerunning Depth Anything and FoundationPose. It contains:

- 40 paired AMBF reset snapshots;
- the DA result metadata;
- the gated FP result table;
- deployment-proxy candidate arrays used to construct the frozen goal bank.

It deliberately excludes RGB/depth frames and is therefore suitable for
**Reach replay**, not for re-evaluating DA image quality. To reproduce the
perception computation itself, run SIM-S3 with the external 3.8 GB DA
checkpoint and FoundationPose image.

Provenance: `SurgicAI_SIM_S3_live_needle_initial_gate_20260811`; validated raw
result was proxy raw flip 1/40, post-gate flip 0/40, accepted 40/40.
