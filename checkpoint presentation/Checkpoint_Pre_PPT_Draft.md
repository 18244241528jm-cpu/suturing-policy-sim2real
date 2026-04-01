# Checkpoint Pre PPT Draft

Project: Adapting Robotic Surgical Suturing Policy from Simulation to Real Environment

Audience goal:
- Show a clear project workflow
- Show current status at checkpoint
- Show measurable deliverables and next steps
- Avoid sounding like a repeated project-plan presentation

Main framing:
- This checkpoint is organized around two parallel workstreams:
- `Track A`: Data / Policy Preparation
- `Track B`: Safe Real-Robot Deployment
- `Key message`: building the ROS2/CRTK bridge is necessary, but not sufficient; meaningful real deployment also requires physical alignment between simulation assumptions and the real setup.
- `Presentation emphasis`: keep both tracks visible, but make deployment the main technical story of this checkpoint.

---

## Slide 1. Title

**Title**
- Adapting Robotic Surgical Suturing Policy from Simulation to Real Environment

**Subtitle**
- Checkpoint Presentation
- CIS II
- Team members + mentors

**Speaker goal**
- Introduce the project in one sentence

**Suggested spoken framing**
- Our checkpoint focuses on two parallel workstreams for Sim2Real suturing: data and policy preparation, and safe deployment on the real dVRK.

**Suggested visual**
- One clean project image only
- Prefer a suturing image, dVRK image, or a simple project graphic

---

## Slide 2. Why This Project Matters

**Title**
- Why This Project Matters

**Slide bullets**
- Simulation-trained suturing policies often fail on real robots because of the Sim2Real gap.
- The gap includes visual mismatch, geometric mismatch, system latency, and physical interaction differences.
- Our goal is to build a practical path from simulation policy to real dVRK deployment.
- This project extends the lab's existing simulation capability toward real-hardware deployment on the dVRK.
- Even before full autonomous suturing success, a safe deployment pipeline is already a valuable outcome.

**Speaker goal**
- Keep this short
- Explain significance without repeating the full project-plan background

**Suggested visual**
- Side-by-side image: simulation vs real dVRK
- Small label in the middle: `Sim2Real Gap`

---

## Slide 3. Checkpoint Deliverables and Current Status

**Title**
- Deliverables and Current Status

**Recommended table**

| Deliverable Tier | Target Outcome | Current Checkpoint Status |
|---|---|---|
| Minimum | Suturing in different simulation configurations (Sim2Sim) | Existing baseline / prior sim-side capability is available; not the main focus of this checkpoint |
| Expected | Automated suturing on one physical dVRK setup (Sim2Real) | Main focus of this checkpoint: deployment bridge and first supervised validation path |
| Stretch | Robustness across varied conditions or multiple physical variants | Deferred until after first deployment and failure analysis |

**Short bullets under table**
- Current checkpoint focus is to de-risk the expected deliverable, not to claim full Sim2Real success yet.
- We are organizing the project into two executable workstreams, but this checkpoint is mainly centered on deployment readiness.
- A working adapter alone is not enough; real execution must also respect the physical camera-tool-robot geometry.
- Our contribution at this stage is the deployment bridge, lab-side verification, and structured failure analysis needed to move from simulation assets toward real dVRK deployment.

**Speaker goal**
- Show exactly where the project stands now
- Make it easy for the instructors to map this talk to checkpoint expectations

**Suggested visual**
- Progress bar or status icons: `Done / In progress / Next`

---

## Slide 4. System Overview: Two Parallel Workstreams

**Title**
- System Overview: Two Parallel Workstreams

**Left side: Track A**
- Teleoperation and demo/data workflow
- Policy input/output understanding
- Policy assets and inference readiness

**Right side: Track B**
- Adapter from policy output to CRTK commands
- ROS2 / dVRK interface setup
- Camera/state topic verification
- Safety checks and validation flow

**Connection statement**
- Track A makes the policy usable and tells us what the deployment stack expects as input and output.
- Track B makes the policy executable at the robot-interface level.
- Meaningful closed-loop behavior still depends on physical alignment between camera, needle, and PSM assumptions.

**Project-plan alignment**
- This preserves the original project-plan logic:
- `simulation baseline -> deployment attempt -> gap identification -> targeted mitigation -> redeployment`

**Speaker goal**
- This should be the main block diagram slide
- Make the whole project feel connected and understandable

**Suggested visual**
- Block diagram:
- `teleoperation / demos / policy prep -> policy model`
- `policy model -> adapter -> ROS2 / CRTK -> dVRK`
- Add side box: `safety + logging`

---

## Slide 5. Track A Progress: Data and Policy Preparation

**Title**
- Track A Progress: Data and Policy Preparation

**Slide bullets**
- We clarified the teleoperation-to-data workflow and how demonstrations are collected.
- We identified the policy input/output structure needed for deployment.
- We reviewed what data, model assets, and interfaces are already available from the lab.
- We clarified that the current Image IL policy is conditioned on both image and 7D proprio input, not image alone.
- We are not making self-training the main focus of this checkpoint.
- If deployment reveals consistent failure modes, the next step would be targeted improvement such as environment adjustment, domain randomization, or domain adaptation with group support.

**Optional stronger wording if accurate**
- Current progress is focused on deployment-facing understanding rather than retraining a new model at this stage.

**Speaker goal**
- Emphasize real progress without overstating training results
- Show that this track is organized and actionable

**Suggested visual**
- Mini workflow:
- `MTM teleoperation -> demos / rosbag -> policy observations/actions -> deployment-ready policy interface`

---

## Slide 6. Track B Progress: Real-Robot Deployment Pipeline

**Title**
- Track B Progress: Real-Robot Deployment Pipeline

**Slide bullets**
- We located a ROS2-compatible `dvrk_policy_adapter` in the SurgicAI Image IL stack.
- The adapter assembles image plus robot state for inference, then maps the 7D policy output into CRTK-style commands.
- This gives us a concrete software path: `image + measured_cp + jaw -> policy -> action -> servo_cp / jaw command`.
- We defined a staged validation path: topic verification -> model loading -> dry validation / read-only checks -> supervised minimal live test.
- Live deployment still depends on lab-side verification of camera topic, `frame_id`, jaw range, safety bounds, and scene consistency with simulation assumptions.

**Contribution note**
- At this checkpoint, our new work is not the original policy itself, but the deployment bridge and the experimental process needed to test it meaningfully on the real system.

**Speaker goal**
- This is the strongest engineering-progress slide
- Make deployment sound concrete, but do not overclaim real-world readiness

**Suggested visual**
- Simple adapter diagram:
- `camera + measured_cp + jaw -> proprio/image -> policy -> action -> adapter -> servo_cp / jaw command`

---

## Slide 7. Current Gaps and Risks

**Title**
- Current Gaps and Risks

**Recommended table**

| Area | Current Gap | Impact |
|---|---|---|
| Data side | It is still unclear how much additional real teleoperation data will be needed for later adaptation. | Does not block the current checkpoint |
| Deployment side | Some live robot interface details still need verification on the actual system. | May delay the first supervised live test |
| Safety side | Full live validation is still pending. | Blocks unsupervised deployment |
| Geometry / Sim2Real side | The physical relationship between camera, needle, and PSM is not yet aligned with the simulation assumptions used by the policy. | A working adapter may still produce meaningless behavior |

**Extra emphasis**
- The main uncertainty is no longer whether we can write the bridge, but whether the real setup is close enough to the policy's training assumptions for the output to be meaningful.

**Speaker goal**
- Show that risks are known and structured
- Do not sound vague or surprised by them

**Suggested visual**
- Risk table only
- Keep it clean and readable

---

## Slide 8. Updated Plan and Recovery Strategy

**Title**
- Updated Plan and Recovery Strategy

**Next 1-2 weeks**
- Verify camera and state topics on the real dVRK setup.
- Validate the adapter in dry run first, then in supervised live testing.
- Log the first real deployment behavior carefully.
- Categorize failures as visual, kinematic, dynamics, geometry, or pipeline-related.

**Recovery logic**
- If the issue is pipeline-related, tune mapping, scaling, bounds, or step size.
- If the issue is geometric, align the physical setup more closely to the simulation assumptions before claiming policy validity.
- If the issue is visual, first adjust the environment before retraining.
- If the gap is larger than expected, use the deployment failure analysis to decide whether to request targeted policy improvement such as DR, environment generalization, or other adaptation work.

**Alignment with original approach**
- This follows the original project-plan sequence:
- naive deployment -> identify the real gap -> choose mitigation -> redeploy

**Speaker goal**
- Show that the project has a recovery path, not just an optimistic plan

**Suggested visual**
- Decision tree:
- `first live test -> failure category -> corresponding mitigation`

---

## Slide 9. Team Roles, Milestones, and Management

**Title**
- Team Roles, Milestones, and Management

**Recommended bullets**
- Member 1: teleoperation/data workflow and policy-side preparation
- Member 2: dVRK adapter and deployment validation
- Shared: experiment logging, mentor review, documentation, and presentation updates
- Near-term milestone 1: verify topics, `frame_id`, jaw range, and model loading
- Near-term milestone 2: complete a supervised minimal live deployment attempt
- Near-term milestone 3: categorize failures and decide whether mitigation should focus on environment, DR, or DA
- Weekly mentor meetings are used to review progress, document blockers, and adjust the near-term plan

**Optional line**
- Documentation and milestone updates are maintained continuously so the project status remains explicit.

**Speaker goal**
- Keep this concrete
- Avoid spending time on generic meeting logistics

**Suggested visual**
- Two-column role split
- Add a small row for shared tasks

---

## Slide 10. Main Takeaways

**Title**
- Main Takeaways

**Slide bullets**
- We restructured the project into two clear and executable workstreams.
- We now have a concrete deployment path, not just a high-level Sim2Real goal.
- The main value of this checkpoint is a minimal closed-loop deployment attempt plus structured failure analysis.
- Our immediate next milestone is a supervised real-robot validation and failure categorization step.

**Closing sentence**
- This checkpoint moves the project from planning toward real deployment execution.

**Speaker goal**
- End on clarity and momentum
- Do not end on references or generic future work

---

## Backup Slide Ideas

Use only if asked.

### Backup A. Adapter Technical Details
- Observation and action definition
- Topic mapping
- Dry-run behavior
- Why the policy is `image + proprio`, not image-only

### Backup A2. Why Deployment Can Still Fail
- Bridge works but geometry mismatches
- Image enters the model but scene semantics change
- Proprio is readable but may not match training assumptions exactly

### Backup B. Safety Checklist
- Bounds
- Step size limit
- Supervised testing only
- Hold / stop behavior

### Backup C. Detailed Timeline
- Week-by-week near-term milestones
- Include owner and expected artifact

### Backup D. References
- SurgicAI
- SRC
- dVRK docs
- CRTK docs

---

## Style Notes From Feedback

- Put one clear workflow diagram early
- State current status explicitly
- Use measurable deliverables
- Keep text lighter than the previous project-plan presentation
- Explain how each component fits into the full project
- If something is delayed, state impact and recovery plan
- Make team roles concrete

---

## Possible Next Revision

In the next revision, this draft can be turned into:
- shorter slide-ready English bullets
- suggested presenter split
- a polished timeline slide
- a final speaking script for each page
