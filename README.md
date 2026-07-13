# Dual-Loop Controlled DC-DC Boost Converter Design

Contains a complete mathematical modeling, control loop design, and dynamic simulation framework for a synchronous DC-DC Boost Converter. The project employs a mathematical script alongside a high-fidelity power-block simulation to demonstrate precise transient regulation and stability.

# Key Features
- State-Space Averaged Modeling: Automated extraction of small-signal control-to-output ($G_{vd}$) and control-to-inductor current ($G_{id}$) transfer functions.
- Dual-Loop Control Topology: High-bandwidth inner current loop paired with a robust outer voltage loop to handle right-half-plane (RHP) zero constraints.
- Type-II Analog Compensator Design: Exact automated component-level calculation ($R$, $C$ values) for targeted phase margins and crossover frequencies.

# Controller Architecture

The system utilizes a Dual-Loop Average Current-Mode Control scheme to achieve optimal transient performance and to inherently protect the switching elements from overcurrent conditions.

1. Outer Voltage Loop ($G_v$): Operates at a crossover frequency of $1\text{ kHz}$ to regulate the output voltage ($V_{out}$) tightly to $24\text{ V}$.
2. Inner Current Loop ($Gi$): Operates at a much faster crossover frequency of $10\text{ kHz}$ to instantly respond to line/load perturbations and stabilize the inductor current ($i_L$).

# Control TargetsTarget
- Phase Margin: $60^\circ$ (Both loops)
- Current Loop Bandwidth ($f_{ci}$): $10\text{ kHz}$
- Voltage Loop Bandwidth ($f_{cv}$): $1\text{ kHz}$

# 1. Simulation & Performance Results

The complete system block diagram is implemented in Simulink, illustrating the distinct power block, dual-loop controller stage, and PWM driver routing:

<img width="2454" height="1117" alt="1" src="https://github.com/user-attachments/assets/98a8ff00-ee34-480b-abcf-829f9d4e9ac0" />

# 2. Startup Transient Response

The system smoothly transitions from the input source voltage up to the regulated $24\text{ V}$ steady state. The inner loop safely guides the inductor current profile without excessive peak spikes:

<img width="1804" height="1988" alt="7" src="https://github.com/user-attachments/assets/8a3ebffb-4156-45dd-a754-8cc5d750a427" />

# 3. Load Regulation & Dynamic Recovery

The design demonstrates excellent load regulation and robustness against step-load disturbances. The compensators efficiently dampen voltage undershoots and overshoots, restoring steady-state regulation seamlessly:

<img width="1818" height="1960" alt="8" src="https://github.com/user-attachments/assets/af81ae81-cf1e-4926-a981-32c031be7942" />
