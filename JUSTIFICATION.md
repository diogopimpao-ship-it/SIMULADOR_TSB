Technical Justification

1. Scope of this implementation

This project implements an Unreal Engine simulator for the OTTER marine vehicle, following the recruitment challenge requirements and extending them with additional functionality where appropriate. The core requested feature is a Fossen-inspired 3DOF marine dynamics model with differential thrust control, while 6DOF dynamics and sensor simulation are presented in the specification as optional but valuable upgrades. :contentReference[oaicite:1]{index=1}

The final implementation, therefore, follows a layered design:

* a 3DOF dynamics model as the baseline solution,
* an optional 6DOF extension for richer vessel motion,
* a sensor simulation layer for GPS, AHRS, and LiDAR,
* a ROS2 communication bridge for publishing simulated sensor outputs.

The overall design goal was not maximum hydrodynamic fidelity, but rather a simulator that is:

* stable in real time,
* controllable,
* modular,
* easy to debug,
* suitable for future navigation and control testing.

---

2. Why is the simulator state manually propagated instead of using Unreal rigid-body physics

A purposeful design decision in this project was to disable the default Unreal rigid-body physics on the hull mesh and propagate the vessel state manually through custom dynamics integration.

This is visible in the constructor and BeginPlay(), where the hull mesh is configured with:

* physics simulation disabled,
* gravity disabled,
* zero linear and angular damping.

Reasoning

This choice was made for several reasons:

1. The challenge is about implementing the vessel dynamics model itself.
If Unreal's rigid-body simulation were used as the primary motion source, the project would rely on a generic engine solver rather than the intended marine equations.
2. Marine dynamics are not equivalent to generic game physics.
The OTTER vehicle model relies on hydrodynamic terms such as added mass, damping, and interconnection effects, which are better represented in a custom simulation loop than in a standard game physics body.
3. The simulator needs a clean relationship between state, sensors, and ROS2 outputs.
Since GPS, AHRS, and LiDAR outputs all depend on the simulated state, it is preferable that all of them originate from a single custom dynamics model rather than mixing custom sensors with engine-driven motion.
4. Manual propagation improves debugging and traceability.
Quantities such as EtaX_m, EtaPsi_rad, NuU_mps, NuR_radps, BodySpecificForceX_mps2, and the sensor measurements are all computed explicitly and can be inspected directly.

For these reasons, the simulation loop was intentionally implemented outside Unreal’s default rigid-body framework.

---

3. Why 3DOF was implemented as the baseline dynamics model

The challenge specification explicitly identifies the 3DOF simplification of the Fossen model as the core required vehicle dynamics model. :contentReference[oaicite:2]{index=2}

For a surface vehicle such as the OTTER, operating mainly in the horizontal plane, the most relevant motion variables for guidance, navigation, and control are:

* surge (u)
* sway (v)
* yaw (r)
* planar pose (x, y, psi)

How is this reflected in the code?

The baseline model uses:

* EtaX_m, EtaY_m, EtaPsi_rad
* NuU_mps, NuV_mps, NuR_radps

and propagates them in StepDynamics(float Dt).

Why was this baseline kept?

This decision is justified because:

1. It directly satisfies the main project requirement.
The problem statement explicitly requests a simulator grounded in the 3DOF simplification. :contentReference[oaicite:3]{index=3}
2. It is the most relevant model for planar maneuvering.
For low-to-moderate speed surface navigation, horizontal-plane dynamics are the primary concern.
3. It provides a stable, interpretable baseline.
Since this project also introduces sensors and ROS2 integration, using a strong 3DOF reference model makes the full simulator easier to validate.
4. It makes later extension easier.
By clearly implementing 3DOF dynamics first, the 6DOF mode can be treated as an upgrade rather than a replacement of the core behavior.

---

4. Why a 6DOF extension was added

The challenge lists 6DOF simulation as an optional, advanced feature and explicitly states that any simplifications introduced should be justified. :contentReference[oaicite:4]{index=4}

For that reason, a 6DOF mode was added in the project through:

* EtaZ_m
* EtaPhi_rad
* EtaTheta_rad
* NuW_mps
* NuP_radps
* NuQ_radps

and implemented in StepDynamics6DOF(float Dt).

Motivation for including it

The 6DOF mode was added because:

1. It demonstrates extension beyond the baseline requirement,
2. It enables richer vessel motion,
3. It creates a better foundation for subsequent interaction with disturbances, sea-state effects, and more realistic attitude sensing.

Important note

Even though the project includes 6DOF, the 3DOF model remains the conceptual baseline, and the 6DOF mode should be interpreted as an extension for increased realism rather than a replacement of the required core model.

---

5. Specific simplifications adopted in the 6DOF model

The 6DOF implementation in this project is intentionally simplified. This was a conscious engineering decision to keep the simulator stable, understandable, and deliverable within the scope of the challenge.

5.1 Decoupled scalar mass / inertia representation

Instead of using the full 6x6 rigid-body and added-mass matrices, the project uses scalar effective terms:

* m11 = Mass + X_u_dot
* m22 = Mass + Y_v_dot
* m33 = Mass + Z_w_dot
* m44 = InertiaRoll + K_p_dot
* m55 = InertiaPitch + M_q_dot
* m66 = InertiaYaw + N_r_dot

Justification

This approach was chosen because:

* It uses the available parameters directly,
* It avoids the complexity of assembling full system matrices,
* It reduces implementation risk,
* It is easier to debug axis by axis.

The resulting model is not a full coupled Fossen implementation, but it is an acceptable simplified extension for a recruitment-scope simulator.

---

5.2 Linear damping only

The code uses linear damping terms such as:

* X_u
* Y_v
* N_r
* Z_w
* K_p
* M_q

and does not implement quadratic damping.

Justification

This decision was made because:

* The specification provides linear damping values explicitly,
* No calibrated quadratic coefficients were available in the provided parameter list.
* Introducing guessed nonlinear damping terms would reduce credibility and complicate tuning,
* Linear damping is sufficient for a first functional simulator.

This holds the implementation closer to the supplied data and avoids unsupported assumptions.

---

5.3 Simplified restorative forces in heave, roll, and pitch

The 6DOF implementation introduces restoring behavior through:

* BuoyancyZ_N
* GravityZ
* GM_T
* GM_L

The restoring terms are modeled as:

* vertical net force from buoyancy and weight,
* roll restoring moment from transverse metacentric height,
* pitch restoring moment from longitudinal metacentric height.

Justification

A full hydrostatic model would require submerged geometry, variation in displaced volume, centers of buoyancy, and potentially nonlinear buoyancy behavior.

That level of detail is outside the scope of this project. Therefore, simplified restoring terms were used in order to:

* capture the basic tendency of the vessel to return to equilibrium,
* allow roll and pitch motion to stay bounded,
* keep the model interpretable and computationally light.

This is especially important because the challenge itself notes that more advanced wave/hydrodynamic modeling would require additional study and is not expected to be fully implemented. :contentReference[oaicite:5]{index=5}

---

5.4 Simplified lift-like terms in heave and pitch

The project includes:

* HeaveLiftCoeff
* PitchLiftCoeff

used to generate forward-speed-dependent contributions in heave and pitch.

Justification

These terms are not meant to represent a full hydrodynamic lift model. They are heuristic additions used to enrich the motion response when the vessel moves forward.

This decision was made because:

* A purely damped 6DOF model can feel too static,
* Adding speed-dependent vertical and pitching effects gives the motion more physical richness,
* The implementation remains easy to tune and computationally cheap.

This is therefore a deliberate approximation to enhance realism, not a claim of full hydrodynamic fidelity.

---

5.5 Simplified Coriolis treatment

The 3DOF model includes explicit Coriolis / centripetal coupling terms for surge, sway, and yaw.
In the 6DOF model, the main coupling emphasis stays on the planar components, while the extra DOFs are treated with simpler force and damping relationships.

Justification

A complete 6DOF Coriolis matrix is significantly more complex and would require a more thorough formulation, more parameters, and more validation work.

Given the scope of the challenge and the fact that 6DOF is optional, the project prioritizes:

* stability,
* readability,
* incremental realism,
rather than attempting a full high-order hydrodynamic implementation without adequate calibration.

---

6. Why a fixed-step integration loop was used

The simulation uses:

* FixedTimeStep
* MaxSubstepsPerTick
* TimeAccumulator

and performs state integration inside a fixed-step loop during Tick().

Justification

This is one of the most important technical choices in the project.

A fixed-step simulation loop was selected because:

1. Numerical calculation becomes more stable, especially when the frame rate varies,
2. sensor update logic becomes easier to manage, since sensors have their own update rates,
3. real-time behavior becomes more consistent, independent of render performance,
4. 3DOF and 6DOF dynamics are easier to compare and debug.

This design is particularly suitable in Unreal Engine, where rendering frame time may fluctuate significantly.

---

7. Propulsion model justification

The challenge requires the use of BlueRobotics T200 thrusters and states that the implementation must follow the official current-to-thrust relationship. :contentReference[oaicite:6]{index=6}

How this was implemented

The project models the propulsion system using:

* commanded current in the range [-20 A, +20 A],
* first-order actuator dynamics through MotorTimeConstant,
* lookup-table interpolation using CalculateT200Thrust() and Interpolate1D().

For forward and reverse thrust, separate lookup tables are used:

* ForwardThrust_N
* ReverseThrust_N

Why this approach was chosen

1. It fulfills the challenge requirement directly.
The thrust model is not an arbitrary approximation; it is implemented from current-to-thrust data.
2. The first-order actuator dynamics avoid unrealistic instantaneous response.
Without them, thrust would jump immediately to target values, and the vehicle would feel physically less plausible.
3. Separate forward and reverse curves to improve realism.
Marine thrusters are not perfectly symmetric in forward and reverse operation, so modeling both directions separately is more credible.
4. Lookup-table interpolation is robust and transparent.
It is easy to verify and modify, and it avoids overfitting a complicated analytical function to discrete performance data.

---

8. Justification for the control mixing strategy

The simulator maps player input into left and right motor commands through:

* InputForward
* InputRight

and mixes them as:

* left = forward - right
* right = forward + right

Justification

This is the standard and most intuitive approach for a differential-thrust vessel controlled by two independent propulsors.

It was selected because:

* It is simple and effective,
* It allows immediate manual testing,
* It reflects the vessel actuation structure directly,
* It fits the project requirement for basic motion control using WASD. :contentReference[oaicite:7]{index=7}

---

9. Why is the actor transform updated from the state instead of being derived from physics bodies

The project uses UpdateActorTransformFromState() to apply the simulated state to the Unreal actor transform.

Justification

This is the natural consequence of manual state integration.

Instead of asking Unreal to infer motion from forces, the simulator computes:

* position,
* heading,
* optional 6DOF orientation,
directly from the vessel model, then applies that transform to the actor.

This improves:

* determinism,
* clarity,
* correspondence between simulation variables and rendered motion.

It also avoids double-simulation effects that could happen if Unreal physics and custom marine equations both tried to influence the same actor.

---

10. Sensor layer design justification

The challenge advocates implementing GPS, AHRS, and LiDAR while accounting for realistic issues such as latency, noise, and sensor failure. :contentReference[oaicite:8]{index=8}

This project implements all three sensors with a common philosophy:

* Each sensor has its own update rate,
* Each sensor can add noise and latency,
* Each sensor can enter persistent health modes,
* Each sensor stores pending measurements before publication.

This is one of the project's strongest parts because it turns the simulator into a useful platform not just for control, but also for estimator and autonomy testing.

---

11. GPS model justification

How GPS is implemented

The GPS output is generated from the transform of the front GPS socket when available.
The measured quantities are:

* GPSMeasuredX_m
* GPSMeasuredY_m

The model includes:

* Gaussian noise (GPSPosSigma_m),
* degraded noise (GPSDegradedPosSigma_m),
* bias random walk (GPSBiasWalkSigma_mps),
* latency (GPSLatency_s),
* latency jitter (GPSLatencyJitter_s),
* health modes (Healthy, Degraded, Frozen, Lost).

Why this design was chosen

1. Socket-based sensing is more realistic than using the actor origin only.
It allows the sensor to be tied to the intended physical sensor location on the mesh.
2. A biased random walk was included to avoid unrealistically memoryless measurements.
Real navigation sensors commonly exhibit a slowly drifting bias.
3. Latency was implemented explicitly because estimator and controller robustness depend on it.
A perfectly synchronous sensor would be too idealized.
4. Frozen and Lost modes were included because they produce failure cases that are very relevant for autonomy systems.
A sensor that remains valid but stale can be just as dangerous as one that becomes invalid.

Fallback behavior

If the GPS socket is not available, the code falls back to the simulated state (EtaX_m, EtaY_m).

This was a deliberate robustness choice.
The simulator should remain functional even if an asset is misconfigured, instead of failing completely due to a missing socket.

---

12. AHRS model justification

How AHRS is implemented

The AHRS provides:

* yaw,
* yaw rate,
* Specific force in the body frame.

The implementation includes:

* gyro noise,
* accelerometer noise,
* gyro bias random walk,
* accelerometer bias random walk,
* latency and jitter,
* health modes.

A key design element is that the AHRS yaw is not taken directly from the exact vessel heading at publication time. Instead:

* The yaw rate is measured with noise and bias.
* Yaw is estimated through integrating that rate,
* A slow correction term is applied using AHRSYawCorrectionGain.

Why this approach was chosen

This is a very important project-specific decision.

A perfect sensor would simply publish EtaPsi_rad, but that would not represent realistic AHRS behavior.
Instead, the project simulates a drifting inertial heading estimate with bounded correction.

This is justified because:

1. It introduces realistic imperfection,
2. It makes the AHRS more meaningful for testing,
3. It is far simpler than implementing a complete INS/AHRS filter,
4. It preserves computational effectiveness.

Frozen and Lost behavior

In Frozen mode, yaw is held at the last healthy estimate, and yaw rate is forced to zero.
In Lost mode, the sample becomes invalid.

This is a good design because it creates clearly distinguishable failure classes:

* degraded but valid,
* valid but stale,
* invalid / unavailable.

Specific force output

The project publishes:

* BodySpecificForceX_mps2
* BodySpecificForceY_mps2

with bias and noise added.

This is preferable to outputting only orientation, as it provides downstream systems with more information and better reflects the role of an inertial sensor.

---

13. LiDAR model justification

How LiDAR is implemented

The LiDAR is modeled as a 2D horizontal scanner using Unreal line traces.
The implementation supports:

* configurable horizontal FOV,
* configurable beam count,
* min/max range,
* measurement noise,
* range quantization,
* beam drops,
* latency,
* health modes.

A sample consists of an array of FLiDARBeam, each containing:

* beam angle,
* measured distance,
* hit flag,
* dropped flag.

Why this approach was chosen

1. Line tracing is the most natural and efficient way to simulate a planar LiDAR in Unreal Engine.
It uses the engine collision system directly.
2. A 2D LiDAR is sufficient for the scope of this project.
Since the main navigation dynamics are planar, a horizontal obstacle scanner is already highly useful.
3. The implementation is still efficient enough to run in real time.
A full 3D LiDAR would be significantly more computationally expensive and outside the expected scope.
4. The beam-level modeling is more informative than a single aggregated distance sensor.
It enables richer downstream perception or avoidance logic.

Degradation behavior

The LiDAR degradation model is not limited to simple range noise. It also includes:

* increased noise when degraded,
* beam drop probability,
* frozen scans,
* lost scans.

This is especially useful because LiDAR failures are often not binary; partial degradation is a realistic and valuable test condition.

Quantization

QuantizeRange() was included to simulate finite range resolution.
This makes the sensor output less artificially perfect and more representative of real digital sensor reporting.

Socket-based placement

The LiDAR uses a named sensor socket for its transform.
This is a better design than assuming the actor origin because the scan should originate from the actual sensor mounting location.

---

14. Why were pending sample queues used for all sensors

GPS, AHRS, and LiDAR all use pending sample arrays:

* GPSPendingSamples
* AHRSPendingSamples
* LiDARPendingSamples

Each sensor:

1. generates a sample at its own update instant,
2. assigns a release time using latency and jitter,
3. stores the sample in a queue,
4. only publishes it when the simulation time reaches that release time.

Justification

This is one of the most technically valuable design choices in the project.

It was chosen because:

* It models sensor latency explicitly,
* It keeps sample generation and publication logically separate,
* It supports asynchronous sensor timing,
* It is more realistic than immediate publication.

This architecture also makes the sensors internally consistent: all of them follow the same latency-aware publication strategy.

---

15. Justification for health-state modeling

Each sensor uses a four-state health model:

* Healthy
* Degraded
* Frozen
* Lost

and transitions into these modes based on probabilities and duration intervals.

Why this model was selected

This approach was chosen because it is:

* simple enough to implement clearly,
* expressive enough to cover relevant failure cases,
* easy to inspect in debug output,
* useful for self-governance robustness testing.

A binary “working / not working” model would be too limited.
In practice, sensors frequently degrade gradually or become stale while still appearing valid. The selected health-state structure better reflects that reality.

---

16. Why was ROS2 communication kept in a separate bridge component

The simulator uses a dedicated UOtter_ROS2UEConnector component for ROS2 publication.

Justification

This separation improves the architecture in several ways:

* Simulation logic remains concentrated on physics and sensing,
* communication logic remains modular,
* future changes to ROS topics or message structures can be isolated,
* The simulator can still run even if ROS2 is not the main focus of a test.

This conforms to good software engineering practices and supports the project's requirement for a structured, maintainable implementation. :contentReference[oaicite:9]{index=9}

---

17. Why was sensor debug visualization included

The project includes detailed on-screen debug information for:

* 6DOF / state values,
* GPS status,
* AHRS status,
* LiDAR status,
and uses world debug lines for LiDAR beams.

Justification

This was added because the project is not only about producing outputs, but also about validating correctness during development.

The debug views help verify:

* sensor timing,
* failure modes,
* beam hits,
* measured vs real values,
* current dynamics mode,
* internal biases and pending queue behavior.

This substantially reduces development and validation time and improves confidence in the simulator's behavior.

---

18. Asset-specific fallback choices

The project attempts to locate the sensor mesh by searching for a mesh component named "Cube", and if not found, falls back to HullMesh.

Justification

This is a practical development-oriented solution.

The intention is:

* to use the dedicated sensor mesh when available,
* But keep the simulator operational if the exact mesh naming differs.

This fallback makes the project more resilient to Blueprint or asset-side differences during iteration.

---

19. Known limitations of this specific implementation

This project does not claim to be a complete high-fidelity hydrodynamic simulator.

Important limitations comprise:

* no wave interaction model,
* no full nonlinear hydrostatics,
* no quadratic damping,
* no full 6DOF Coriolis matrix,
* no geodetic GPS model,
* no full navigation-grade AHRS filter,
* LiDAR is limited to a planar scan.

These are acceptable restrictions in the context of the project because the main objective was to deliver a functioning, readable, and extensible simulator within the challenge scope. :contentReference[oaicite:10]{index=10}

---

20. Final justification summary

The implementation choices made in this project consistently favor:

* correctness relative to the project scope,
* clarity of code structure,
* real-time stability,
* ease of validation,
* extensibility.

The simulator does not attempt to maximize physical complexity at all costs.
Instead, it intends to provide a technically sound springboard for future development, especially in the areas of:

* control,
* estimation,
* sensor processing,
* ROS2 integration,
* more advanced marine simulation features.

In that sense, every major simplification in the project was intentional:
not to avoid effort, but to keep the solution robust, understandable, and appropriate for the challenge objectives.

