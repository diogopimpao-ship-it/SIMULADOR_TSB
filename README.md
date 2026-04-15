OTTER Marine Simulator



Unreal Engine + ROS2



Overview



This project builds a marine vehicle simulator for the OTTER platform with Unreal Engine.

The simulator was made for the Autonomous Systems recruitment challenge and uses a marine dynamics model based on Fossen.



The main goal is to provide a flexible simulation environment for safely testing navigation, estimation, and control algorithms.



The implementation includes:



* 3DOF vessel dynamics as the main baseline model

* optional 6DOF extension

* differential thrust propulsion using BlueRobotics T200 thrusters

* keyboard control with WASD

* simulated GPS, AHRS, and LiDAR devices

* sensor interference, latency, jitter, and fault modes

* ROS2 publishing of sensor data



Project Goals



According to the project specification, the simulator should provide:



* An Unreal Engine marine simulation environment

* a Fossen-based vehicle model

* differential thrust propulsion

* basic vessel control

* optional sensor simulation and ROS2 integration :contentReference[oaicite:1]{index=1}



This implementation meets those requirements and keeps the code modular and easy to expand.



Implemented Features



1. Vehicle Dynamics



The simulator models OTTER vessel dynamics with a manually integrated state-space approach.



3DOF baseline



The main model uses the horizontal-plane simplification:



* pose: x, y, psi

* body velocities: u, v, r



The model includes:



* rigid-body inertia

* added mass

* linear damping

* Coriolis/centripetal coupling terms

* differential thrust actuation



This model meets the main dynamics requirement for the challenge. :contentReference[oaicite:2]{index=2}



Optional 6DOF extension



An optional 6DOF mode adds heave, roll, and pitch to the simulation. This extends the state with these extra motions.



This mode adds simplified:



* heave dynamics

* roll and pitch dynamics

* restorative forces and moments

* lift-like effects



You can find more details about the simplified 6DOF implementation in the technical justification document.



2. Propulsion System



The vehicle uses two BlueRobotics T200 thrusters in a differential-thrust setup, as required by the challenge. :contentReference[oaicite:3]{index=3}



Implemented features:



* Current commands range from -20 A to +20 A

* first-order actuator dynamics

* lookup-table interpolation from current to thrust

* left/right thrust mixing from user input



3. User Control



You can control the simulator using the keyboard:



* W / S → forward / reverse thrust

* A / D → differential steering



These inputs are converted into current commands for the left and right motors.



4. Sensor Simulation



The simulator has a sensor simulation layer, as described in the project statement. :contentReference[oaicite:4]{index=4}



GPS



The GPS sensor provides local Cartesian position estimates:



* X_m

* Y_m



Features:



* Gaussian measurement noise

* slow bias random walk

* latency

* latency jitter

* persistent fault modes



Fault modes:



* Healthy

* Degraded

* Frozen

* Lost



AHRS



The AHRS sensor provides:



* yaw

* yaw rate

* specific force in body-frame x and y



Features:



* gyro and accelerometer noise

* gyro and accelerometer bias random walk

* latency

* latency jitter

* persistent fault modes



Fault modes:



* Healthy

* Degraded

* Frozen

* Lost



LiDAR



The LiDAR sensor acts as a 2D horizontal scanner using Unreal line traces. Features:



* configurable field of view

* configurable number of beams

* configurable min/max range

* Gaussian range noise

* range quantization

* beam drop probability

* latency

* latency jitter

* persistent fault modes



Fault modes:



* Healthy

* Degraded

* Frozen

* Lost



5. ROS2 Integration



The simulator connects to ROS2 using the UOtter_ROS2UEConnector component. Sensor data is published to ROS2 (Robot Operating System 2), so external autonomy modules can subscribe in real time, as required by the project specification.



Published data includes:



* GPS

* AHRS

* LiDAR



Project Structure



A simplified overview of the main files:



* Otter_Pawn.h defines the pawn and simulation state variables. It also contains sensor structures, adjustable parameters, and public or protected methods.

* Otter_Pawn.cpp

It handles vessel dynamics, sensor simulation, debug drawing, user input, and ROS2 publishing.

* Otter_ROS2UEConnector.*

This file manages communication between the Unreal simulator and ROS2.



The simulator does not use Unreal Engine's rigid-body physics for vessel dynamics.



Instead:



* Physics simulation is disabled on the hull mesh.

* The vessel state is propagated manually.

* The actor transform is updated from the simulated state.



This approach provides full control over the mathematical model and maintains simulation consistency with the intended marine dynamics.



Fixed-step integration



The simulation uses:



* FixedTimeStep

* MaxSubstepsPerTick: This setup improves numerical accuracy and reduces dependence on frame rate.



Sensor latency model



Each sensor generates samples at its own update rate.

These samples are placed in a queue and released after their latency expires.



This allows the simulator to model:



* delayed measurements

* sensor-specific update frequencies

* asynchronous publication timing



Vehicle Parameters



The implementation uses the OTTER parameters specified in the challenge description, including rigid-body inertia, added mass, damping, and propulsion constants. :contentReference[oaicite:6]{index=6}



Main parameters in the code include:



* Mass

* InertiaYaw

* X_u_dot, Y_v_dot, N_r_dot

* X_u, Y_v, N_r

* ThrusterHalfBeam

* MaxCommandCurrent

* MotorTimeConstant



The 6DOF mode also uses:



* roll/pitch inertias

* heave/roll/pitch added-mass terms

* restoring terms

* simplified lift coefficients



How to Build and Run



Requirements



* Unreal Engine project configured with C++ support.

* Visual Studio / C++ toolchain for Unreal

* Make sure the OTTER mesh and related assets are imported into the project

* Install ROS2 only if you plan to use ROS communication



Steps



1. Open the Unreal Engine project.

2. Build the C++ project.

3. Make sure the OTTER pawn is placed in the level.

4. Ensure the sensor mesh sockets are properly set up:

  * GPSFront

  * GPSBack

5. Ensure sockets for AHRS and Lidar are configured. Press Play in Unreal Editor.

6. Use WASD to control the vessel.



Important Configurable Parameters



You can adjust most of the simulator's behavior directly in Unreal using exposed properties.



Dynamics



* FixedTimeStep

* MaxSubstepsPerTick

* Mass

* InertiaYaw

* damping coefficients

* added-mass coefficients



Thrusters



* MaxCommandCurrent

* MotorTimeConstant

* ThrusterHalfBeam

* ThrusterXOffset_m

* ThrusterZOffset_m



GPS



* GPSUpdateRateHz

* GPSPosSigma_m

* GPSDegradedPosSigma_m

* GPSLatency_s

* GPSLatencyJitter_s

* fault probabilities and durations



AHRS



* AHRSUpdateRateHz

* AHRSYawRateSigma_radps

* AHRSSpecificForceSigma_mps2

* AHRSLatency_s

* AHRSLatencyJitter_s

* Bias walk parameters

* fault probabilities and durations



LiDAR



* LiDARUpdateRateHz

* LiDARHorizontalFOV_deg

* LiDARNumBeams

* LiDARMinRange_m

* LiDARMaxRange_m

* LiDARRangeSigma_m

* LiDARRangeResolution_m

* beam drop probabilities

* fault probabilities and durations



Debug Features



The project includes on-screen debug output for the 6DOF state.



* GPS measurements and health

* AHRS measurements and health

* LiDAR status and beam counts



The LiDAR can also draw debug lines in the world.



Limitations



This simulator was designed to be clear, modular, and functional for the recruitment challenge, rather than to provide full hydrodynamic accuracy.



Current restrictions include:



* no wave model

* no advanced fluid dynamic coupling matrices

* no quadratic damping terms

* simplified 6DOF restoring and lift effects

* LiDAR implemented as 2D horizontal scanning only.

* GPS implemented in local Cartesian coordinates only

* AHRS implemented as a simplified sensor model, not a full navigation-grade estimator



Future Work



Probable next steps include:



* wave interaction model

* improved 6DOF hydrodynamics

* quadratic damping terms

* more realistic sensor models

* standardized ROS2 message definitions

* more complicated scenarios and environment assets



Deliverables Alignment



The project is organized to meet the recruitment deliverables, including:



* top-level README with quickstart/build/run information

* The documentation explains implementation choices that were not fully specified in the challenge.



Author Notes



The simulator was developed with these priorities in mind:



* modular architecture

* readability

* extensibility

* compatibility with future control and navigation modules

