2D Proportional Navigation Guidance Simulation (AIM-9M)

A 2D physics and guidance simulation in Python modeling an AIM-9M Sidewinder pursuing an evasively maneuvering target using Proportional Navigation (PN). The simulation incorporates a boost-coast solid rocket motor profile, quadratic aerodynamic drag, structural G-force saturation, and velocity-mapped telemetry visualization.

1. Quick Start & Reproducibility

Dependencies

The project requires only numpy and matplotlib alongside Python's standard math library.

pip install numpy==2.1.3 matplotlib==3.9.2


Run the Simulation

Clone the repository and run main.py to execute the physics loop, print $50\text{ Hz}$ telemetry logs to the console, and generate the trajectory plot:

python main.py


2. Mathematical Formulation (Whiteboard Derivation)

Relative Kinematics

Given the pursuer and target position and velocity vectors, the relative position vector (Line of Sight), relative velocity vector, and scalar range $R$ are defined as:

$$\vec{r} = \vec{r}_T - \vec{r}_P$$

$$\vec{v}_{\text{rel}} = \vec{v}_T - \vec{v}_P$$

$$R = \vert{}\vec{r}\vert{} = \sqrt{r_x^2 + r_y^2}$$

Closing Velocity

Closing velocity is the rate at which the scalar range $R$ decreases, computed as the negative dot product of the relative position and relative velocity divided by scalar distance:

$$V_c = -\dot{R} = -\frac{\vec{r} \cdot \vec{v}_{\text{rel}}}{R}$$

Line-of-Sight (LOS) Rate

By embedding the 2D vectors into 3D with zero $z$-components, the angular rotation rate $\dot{\lambda}$ of the line-of-sight vector is the $z$-component of their cross product divided by range squared:

$$\vec{r}_{3D} = [r_x, \, r_y, \, 0]^T, \quad \vec{v}_{3D} = [v_{\text{rel},x}, \, v_{\text{rel},y}, \, 0]^T$$

$$\dot{\lambda} = \frac{(\vec{r}_{3D} \times \vec{v}_{3D})_z}{R^2} = \frac{r_x v_{\text{rel},y} - r_y v_{\text{rel},x}}{r_x^2 + r_y^2}$$

On an ideal collision course with a non-maneuvering target, $\dot{\lambda} \to 0$.

Proportional Navigation Steering Law & Terminal Gate

When the missile is actively closing on the target ($V_c > 0$) or within the terminal homing envelope ($R < 100\text{ m}$), the lateral acceleration command magnitude is proportional to the closing velocity and the LOS rate, scaled by the navigation constant $N = 4.0$:

$$a_{\text{lat}} = N \cdot V_c \cdot \dot{\lambda}$$

The lateral acceleration magnitude is clamped by the missile's structural G-limit ($30.0\text{ G}$):

$$\vert{}a_{\text{lat}}\vert{} \le G_{\max} \cdot g_0, \quad G_{\max} = 30.0, \quad g_0 = 9.81\text{ m/s}^2$$

To enforce Zero-Work Turning (Pure PN), the lateral steering vector is applied strictly perpendicular to the pursuer's current velocity unit vector $\hat{u}_v$ so steering changes heading without artificially adding kinetic energy:

$$\hat{u}_v = \frac{\vec{v}_P}{\vert{}\vec{v}_P\vert{}}, \quad \hat{u}_{v,\perp} = [-u_{v,y}, \, u_{v,x}]^T$$

$$\vec{a}_{\text{lateral}} = a_{\text{lat}} \, \hat{u}_{v,\perp}$$

Longitudinal Propulsion & Aerodynamic Drag

Along the missile's heading unit vector $\hat{u}_v$, longitudinal acceleration is governed by solid rocket motor thrust $T(t)$ and parasitic aerodynamic drag $D$:

Boost Phase ($t \le 5.2\text{ s}$): $T(t) = 12,800\text{ N}$

Coast Phase ($t > 5.2\text{ s}$): $T(t) = 0\text{ N}$

$$D = \frac{1}{2} \rho \vert{}\vec{v}_P\vert{}^2 C_d A$$

$$\vec{a}_{\text{longitudinal}} = \left(\frac{T(t) - D}{m}\right)\hat{u}_v$$

$$\vec{a}_{\text{net}} = \vec{a}_{\text{longitudinal}} + \vec{a}_{\text{lateral}}$$

3. Code Architecture

main.py: Configures the simulation timestep ($\Delta t = 0.001\text{ s}$), vehicle initial states, AIM-9M physical constants, and the target's $10\text{G}$-clamped sinusoidal weave:

$$a_x(t) = 80\sin(0.8t), \quad a_y(t) = 60\cos(0.6t)$$

Steps both vehicles forward via .tick(dt), evaluates termination against blast_radius = 10.0 m and cutoff_time = 30 s, and launches the plotter.

utility.py: Contains core guidance and physics functions:

calculate_los_rate(state): Computes $\dot{\lambda}$ via the 3D cross product np.cross(r_3d, v_3d)[2] / distance_squared.

calculate_closing_velocity(state): Computes closing speed $V_c$ via -np.dot(distance_vector, v_relative) / abs_distance.

calculate_dynamic_acceleration(state): Computes gated PN lateral acceleration, applies the $30\text{G}$ structural clamp, adds boost-coast thrust and quadratic drag, and returns the 2D net acceleration vector.

run_log, closing_dist, pol2cart, normalize_angle, error_vector: Telemetry printing and geometric utilities.

vehicle.py: Defines the Vehicle class, logging state snapshots (timestamp, pos, v, a, hdg) to self.history and advancing position and velocity via explicit forward Euler integration.

plotter.py: Extracts trajectory arrays from target.history and pursuer.history, colors the pursuer path by speed using matplotlib.collections.LineCollection, marks the $5.2\text{ s}$ motor burnout point, and calculates and annotates the Closest Point of Approach (CPA).

4. Simulation Results & Endgame Analysis

Engagement Parameters

Pursuer (AIM-9M Model):

Initial Position: $[0.0, \, 0.0]\text{ m}$

Initial Velocity: $200\text{ m/s}$ at $45^\circ$

Navigation Constant ($N$): $4.0$

Mass ($m$): $85.5\text{ kg}$

Max Structural Load: $30.0\text{ G}$ ($294.3\text{ m/s}^2$)

Motor Thrust & Burn Time: $12,800\text{ N}$ for $5.2\text{ s}$

Drag Coefficient ($C_d$): $0.3$

Reference Area ($A$): $0.05067\text{ m}^2$

Air Density ($\rho$): $1.225\text{ kg/m}^3$

Target (Evasive Aircraft):

Initial Position: $[2000.0, \, 2000.0]\text{ m}$

Initial Velocity: $300\text{ m/s}$ at $45^\circ$

Max Structural Load: $10.0\text{ G}$ ($98.1\text{ m/s}^2$)

Maneuver Profile: Continuous sinusoidal weave

Why PN Degrades Against a Maneuvering Target

During the engagement, the missile accelerates past $800\text{ m/s}$ during the $5.2\text{ s}$ boost phase before aerodynamic drag slows it during the coast phase, ultimately achieving a Closest Point of Approach (CPA) of $6.56\text{ meters}$ (within the $10.0\text{ m}$ lethal blast radius of the AIM-9M's WDU-17/B annular fragmentation warhead). Three physical factors prevent a zero-meter point impact:

Continuous Target Maneuvering: Because the target applies a continuous $10\text{G}$-limited sinusoidal weave, the collision triangle constantly shifts and $\dot{\lambda}$ never settles to zero.

Post-Burnout Energy Bleed: Once thrust cuts off at $t = 5.2\text{ s}$, parasitic drag ($D \propto v^2$) bleeds airspeed over the remaining flight, reducing closing speed.

Terminal Closing Speed Decay: In the final meters of a close pass, the relative position vector $\vec{r}$ becomes nearly orthogonal to the relative velocity vector, causing closing speed to drop toward zero right at closest approach:

$$V_c = -\frac{\vec{r} \cdot \vec{v}_{\text{rel}}}{R} \to 0 \implies a_{\text{lat}} = N \cdot V_c \cdot \dot{\lambda} \to 0$$

Even with the $< 100\text{ m}$ terminal override gate keeping steering active, the proportional guidance command naturally loses authority as $V_c \to 0$.

5. Model Assumptions

2D Point-Mass Kinematics: Both vehicles are modeled as 2-DOF point masses without pitch/yaw rotational inertia, angle-of-attack trim drag, or lift-induced drag.

Ideal Seeker & Instantaneous Autopilot: Line-of-sight rate and closing velocity are known instantaneously without sensor noise, gimbal limits, or control actuator lag.

Constant Mass & Sea-Level Density: Vehicle mass ($85.5\text{ kg}$), drag coefficient ($C_d = 0.3$), and air density ($\rho = 1.225\text{ kg/m}^3$) remain constant throughout the flight.

Forward Euler Integration: Kinematics are integrated using a fixed timestep of $\Delta t = 0.001\text{ s}$.