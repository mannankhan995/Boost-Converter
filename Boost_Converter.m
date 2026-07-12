close all; clear; clc
% --- Boost Converter Design Parameters ---
Vin = 12;      % Input Voltage (V)
Vout_des = 24; % Desired Output Voltage (V)
R = 10;        % Load Resistance (Ohm)
L = 100e-6;    % Inductance (H)
C = 220e-6;    % Capacitance (F)
fsw = 100e3;   % Switching Frequency (Hz)

% Calculate Duty Cycle for Ideal Boost
D = 1 - (Vin / Vout_des);

% --- State 1: Switch ON ---
% Inductor charges from Vin, Cap discharges to Load
A1 = [0, 0; 
      0, -1/(R*C)];
B1 = [1/L; 
      0];
C1 = [0, 1]; % Output is v_C
D1 = 0;

% --- State 2: Switch OFF ---
% Inductor discharges to Cap/Load
A2 = [0,    -1/L; 
      1/C, -1/(R*C)];
B2 = [1/L; 
      0];
C2 = [0, 1]; % Output is v_C
D2 = 0;

% --- Run the General Function ---
[sys_avg, Ess_matrix, Gvd, Gid] = get_converter_model(A1, B1, C1, D1, A2, B2, C2, D2, D, Vin);

% Display the calculated Ess Matrix
disp('Calculated Ess Matrix:');
disp(Ess_matrix);

% Display the Transfer Function
disp('Gvd Transfer Function:');


%%

% Define inputs
fc_i = 10e3;
PM_desired = 60;
R1 = 10000; 

[mag_i, phase_i] = bode(Gid, 2*pi*fc_i);
mag_i = 20*log10(mag_i);          
         

% Call the function
[Gi, freqs, comps] = Type_II_Comp(fc_i, PM_desired, mag_i, phase_i, R1);

% Display the results
disp('--- Frequencies (Hz) ---');
disp(freqs);
disp('--- Components (Ohms, Farads) ---');
disp(comps);

% Plot the Bode plot of the compensator to verify it
figure;
opts = bodeoptions;
    opts.FreqUnits = 'Hz';     % Set frequency units to Hertz
    opts.Grid = 'on';
bode(Gi * Gid, opts);
grid on;
title('Type-2 Compensator Transfer Function (Gi)');

figure;
bode(Gid, 1/Gi, opts);
grid on;
title('Inverse Compensator Transfer Function (Gi)');



%% --- Outer Voltage Loop Design ---
fc_v = 1000;
PM_desired = 60;
R1_v = 10000; 

% Define Modulator and Sensor Gains (Set to 1 for ideal math, change for hardware)
Fm = 1;  % PWM Modulator Gain (1/Vramp)
Hi = (50e-3 * 1) * 20;  % Current Sensor Gain (V/A)
Hv = 1;  % Voltage Sensor Gain (V/V)

% 1. Calculate the physical Gvi (Inductor Current to Output Voltage)
Gvi = minreal(Gvd / Gid);

% 2. Calculate the inner closed loop (incorporating Fm and Hi)
T_open_i = Gi * Gid * Fm * Hi;
T_closed_i = feedback(T_open_i, 1);

% 3. Calculate the True Plant for the Voltage Loop
Plant_v = minreal(T_closed_i * Gvi * (1/Hi)); % Divide out Hi so plant expects Amps

% 4. Read Bode parameters at fc_v
[mag_v_linear, phase_v_raw] = bode(Plant_v, 2*pi*fc_v);
mag_v = 20*log10(mag_v_linear);

% Robust Phase Unwrapping: Forces phase into -180 to +180 bounds
phase_v = mod(phase_v_raw, 360);
if phase_v > 180
    phase_v = phase_v - 360;
end

% 5. Design Voltage Compensator
[Gv, freqs_V, comps_V] = Type_II_Comp(fc_v, PM_desired, mag_v, phase_v, R1_v);

%% --- Final System Assembly and Plotting ---

% Calculate Outer Open-Loop (Tv)
Tv = minreal(Gv * Plant_v * Hv);

% Calculate Final Closed-Loop (v_o / v_ref)
T_closed_overall = feedback(Tv, 1);

% Plot 1: Outer Loop Stability
figure;
bode(Tv,opts);
grid on;
title('Outer Voltage Loop (Tv) - Verifying PM and Bandwidth');

% Plot 2: Final Step Response
figure;
step(T_closed_overall);
grid on;
title('Dual-Loop Step Response: Output Voltage');
ylabel('Normalized Voltage');
%%
% f_RHZ = (R * (1 - D)^2)/(2*pi*L);









%%

function [sys_avg, Ess, Gvd, Gid] = get_converter_model(A1, B1, C1, D1, A2, B2, C2, D2, D, Vin)
    % GET_CONVERTER_MODEL Calculates Averaged and Small-Signal models
    %
    % Inputs:
    %   A1, B1... : Matrices for Switch ON (State 1)
    %   A2, B2... : Matrices for Switch OFF (State 2)
    %   D         : Steady state duty cycle (0 to 1)
    %   Vin       : Steady state input voltage
    %
    % Outputs:
    %   sys_avg   : The average large-signal state-space model (ss object)
    %   Ess       : The Control Input Matrix (B matrix for duty cycle)
    %   Gvd       : Transfer function from Duty Cycle to Output Voltage
    %   Gid       : Transfer function from Duty Cycle to Inductor Current

    %% 1. Calculate Averaged Matrices
    % Model: dx/dt = A*x + B*u
    %        y     = C*x + D*u
    A = D*A1 + (1-D)*A2;
    B = D*B1 + (1-D)*B2;
    C = D*C1 + (1-D)*C2;
    E = D*D1 + (1-D)*D2; % 'D' matrix (feedforward), named E here to avoid confusion with Duty 'D'

    sys_avg = ss(A, B, C, E);

    %% 2. Solve for Steady-State Operating Point (X_eq)
    % At DC equilibrium: 0 = A*X + B*Vin
    % Therefore: X = -inv(A) * B * Vin
    X_eq = -inv(A) * B * Vin;
    
    fprintf('Steady State Operating Point:\n');
    fprintf('Inductor Current (IL): %.4f A\n', X_eq(1));
    fprintf('Capacitor Voltage (Vc): %.4f V\n', X_eq(2));

    %% 3. Calculate Small-Signal Matrices (The Perturbation Model)
    % We want the transfer function from Duty Cycle (d_hat) to States (x_hat)
    % Equation: dx_hat/dt = A*x_hat + Ess*d_hat
    
    % Ess = (A1 - A2)*X_eq + (B1 - B2)*Vin
    Ess = (A1 - A2)*X_eq + (B1 - B2)*Vin;
    
    % Output Matrix for Duty Cycle (Feedforward from d to y)
    % Fss = (C1 - C2)*X_eq + (D1 - D2)*Vin
    Fss = (C1 - C2)*X_eq + (D1 - D2)*Vin;

    %% 4. Create Small-Signal System (Input = Duty Cycle)
    % We create a system where Input is d_hat and Outputs are [i_L; v_C]
    % We force C to be identity [1 0; 0 1] to extract both states directly
    C_states = eye(2); 
    D_states = [0; 0];
    
    sys_small_signal = ss(A, Ess, C_states, D_states);
    
    % Extract Transfer Functions
    % Output 1 is Inductor Current (x1), Output 2 is Capacitor Voltage (x2)
    Gid = tf(sys_small_signal(1,1)); % Transfer function: d -> i_L
    Gvd = tf(sys_small_signal(2,1)); % Transfer function: d -> v_C

    %% 5. Plotting
    figure;

    opts = bodeoptions;
    opts.FreqUnits = 'Hz';     % Set frequency units to Hertz
    opts.Grid = 'on';

    bode(Gvd, Gid, opts);
    legend('Gvd (Voltage)', 'Gid (Current)');
    title('Small-Signal Frequency Response (Control-to-Output)');
end





function [Gi, freqs, comps] = Type_II_Comp(fc, PM_desired, mag_fc_dB, phase_fc, R1)
    % Type_II_Comp: Calculates components and transfer function for a Type-2 Compensator
    %
    % Inputs:
    %   fc         - Desired crossover frequency (Hz)
    %   PM_desired - Desired Phase Margin (Degrees)
    %   mag_fc_dB  - Plant magnitude at fc (dB)
    %   phase_fc   - Plant phase at fc (Degrees)
    %   R1         - Chosen input resistor value (Ohms)
    %
    % Outputs:
    %   Gi         - Transfer function object of the compensator
    %   freqs      - Structure containing pole/zero frequencies
    %   comps      - Structure containing R and C component values

    %% 1. Convert Magnitude from dB to Linear Gain
    plant_gain_linear = 10^(mag_fc_dB / 20);
    
    % Compensator must provide the inverse gain to force 0 dB at fc
    comp_gain_linear = 1 / plant_gain_linear;

    %% 2. Calculate Required Phase Boost
    % Formula: Boost = PM_desired - phase_fc - 90
    % (The -90 comes from the origin pole/integrator)
    boost_deg = PM_desired - phase_fc - 90;
    
    % Check if boost is within the physical limits of a Type II (0 to 90 deg)
    if boost_deg <= 0 || boost_deg >= 90
        warning('Required phase boost is %.2f°. A Type-2 compensator can only provide between 0° and 90° of boost.', boost_deg);
    end
    
    %% 3. Calculate K-Factor and Pole/Zero Placement
    boost_rad = deg2rad(boost_deg);
    K = tan((boost_rad / 2) + (pi / 4));
    
    fz = fc / K;
    fp = fc * K;

    %% 4. Calculate Component Values
    R2 = R1 * comp_gain_linear;
    C1 = 1 / (2 * pi * fz * R2);
    C2 = 1 / (2 * pi * fp * R2);

    %% 5. Pack Outputs into Structures
    freqs.fz = fz;
    freqs.fp = fp;
    freqs.fc = fc;

    comps.R1 = R1;
    comps.R2 = R2;
    comps.C1 = C1;
    comps.C2 = C2;

    %% 6. Create the Transfer Function Gi(s)
    % Using the exact component impedance formula:
    % Gc(s) = (1 + s*R2*C1) / (s*R1*(C1+C2) + s^2*R1*R2*C1*C2)
    num = [R2*C1, 1];
    den = [R1*R2*C1*C2, R1*(C1+C2), 0];
    
    Gi = tf(num, den);
end