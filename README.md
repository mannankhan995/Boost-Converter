# Dual-Loop Controlled DC-DC Boost Converter Design

Contains a complete mathematical modeling, control loop design, and dynamic simulation framework for a synchronous DC-DC Boost Converter. The project employs a mathematical script alongside a high-fidelity power-block simulation to demonstrate precise transient regulation and stability.

# Key Features
- State-Space Averaged Modeling: Automated extraction of small-signal control-to-output ($G_{vd}$) and control-to-inductor current ($G_{id}$) transfer functions.
- Dual-Loop Control Topology: High-bandwidth inner current loop paired with a robust outer voltage loop to handle right-half-plane (RHP) zero constraints.
- Type-II Analog Compensator Design: Exact automated component-level calculation ($R$, $C$ values) for targeted phase margins and crossover frequencies.

# Controller Architecture

The system utilizes a Dual-Loop Average Current-Mode Control scheme to achieve optimal transient performance and to inherently protect the switching elements from overcurrent conditions.

<img width="2454" height="1117" alt="1" src="https://github.com/user-attachments/assets/98a8ff00-ee34-480b-abcf-829f9d4e9ac0" />

