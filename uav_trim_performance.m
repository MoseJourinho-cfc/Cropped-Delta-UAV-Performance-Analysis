% =========================================================================
% STUDENT PROJECT ASSIGNMENT 4:
% Project By: Neelay Kavlekar
% Trim, Thrust Required, and Power Required Analysis of a Cropped Delta
% Wing UAV in Steady Level Flight
% =========================================================================
% File: uav_trim_performance.m
% Description:
%   This modular MATLAB script performs a coupled longitudinal trim and
%   steady level-flight performance analysis for a wing-alone cropped delta
%   reflex UAV across an angle-of-attack sweep from 0 to 12 degrees
%   (step size = 0.5 deg).
%
%   The code is structured into three phases matching the project tasks:
%     Phase 1 - Geometric Preprocessing & Iterative Trim/Equilibrium Solver
%     Phase 2 - Aerodynamic Efficiency & Performance Metrics Evaluation
%     Phase 3 - Automated Multi-Figure Visualization & Tabular Summary
% =========================================================================

clear; clc; close all;

%% ========================================================================
%  1. GIVEN UAV CONFIGURATION, AERODYNAMIC, AND FLIGHT CONDITION DATA
%  ========================================================================

% --- Wing-Alone Cropped Delta Geometry ---
Cr = 0.90;          % Root chord [m]
Ct = 0.15;          % Tip chord [m]
b  = 1.50;          % Wingspan [m]
e  = 0.89;          % Oswald span efficiency factor [-]

% --- Longitudinal Aerodynamic Coefficients (per radian where applicable) ---
CD0 = 0.030;        % Zero-lift parasite drag coefficient [-]
CL0 = 0.000;        % Zero-angle-of-attack lift coefficient [-] (symmetric/reflex baseline)
Cm0 = 0.010;        % Zero-lift pitching moment coefficient [-] (positive for reflex section)
CLa = 2.920;        % Lift curve slope, dCL/d(alpha) [1/rad]
CLe = 0.265;        % Elevator lift control derivative, dCL/d(delta_e) [1/rad]
Cma = -0.292;       % Longitudinal static stability derivative, dCm/d(alpha) [1/rad]
Cme = -0.400;       % Elevator pitch control authority derivative, dCm/d(delta_e) [1/rad]

% --- Flight Condition Parameters (Sea Level, Steady Level Flight) ---
m   = 3.50;         % UAV total mass [kg]
rho = 1.225;        % Sea-level standard air density [kg/m^3]
g   = 10.0;         % Gravitational acceleration [m/s^2]
W   = m * g;        % Aircraft weight [N]

%% ========================================================================
%  2. PREPROCESSING CALCULATIONS (GEOMETRY & DRAG POLAR)
%  ========================================================================

% 1. Taper Ratio (lambda)
lambda = Ct / Cr;

% 2. Planform Reference Area (S) [m^2]
S = (b / 2) * Cr * (1 + lambda);

% 3. Wing Aspect Ratio (AR) [-]
AR = (b^2) / S;

% 4. Induced Drag Correction Factor (k) [-]
k = 1 / (pi * e * AR);

% --- Trimmed Lift Curve Slope (for analytical cross-check) ---
CLa_trim = CLa - CLe * (Cma / Cme);

fprintf('======================================================================\n');
fprintf('  CROPPED DELTA WING UAV: PREPROCESSING & GEOMETRIC PARAMETERS\n');
fprintf('======================================================================\n');
fprintf('  Root Chord (Cr)                 : %8.4f m\n', Cr);
fprintf('  Tip Chord (Ct)                  : %8.4f m\n', Ct);
fprintf('  Wingspan (b)                    : %8.4f m\n', b);
fprintf('  Taper Ratio (lambda = Ct/Cr)    : %8.4f (-)\n', lambda);
fprintf('  Planform Area (S)               : %8.4f m^2\n', S);
fprintf('  Aspect Ratio (AR = b^2/S)       : %8.4f (-)\n', AR);
fprintf('  Oswald Efficiency Factor (e)    : %8.4f (-)\n', e);
fprintf('  Induced Drag Factor (k)         : %8.5f (-)\n', k);
fprintf('  UAV Weight (W = m*g)            : %8.4f N\n', W);
fprintf('  Trimmed Lift Slope (CLa_trim)   : %8.4f 1/rad\n', CLa_trim);
fprintf('======================================================================\n\n');

%% ========================================================================
%  3. PHASE 1 & PHASE 2: ITERATIVE TRIM AND PERFORMANCE SOLVER
%  ========================================================================

% Define Angle of Attack sweep: 0 deg to 12 deg with 0.5 deg increment
alpha_deg = (0 : 0.5 : 12.0)';          % Angle of attack vector [deg]
N = length(alpha_deg);                  % Number of discrete flight states

% Preallocate arrays for computational efficiency
alpha_rad    = zeros(N, 1);             % Angle of attack [rad]
de_trim_rad  = zeros(N, 1);             % Trim elevator deflection [rad]
de_trim_deg  = zeros(N, 1);             % Trim elevator deflection [deg]
CL_trim      = zeros(N, 1);             % Trimmed lift coefficient [-]
V_flight     = zeros(N, 1);             % Steady level flight velocity [m/s]
q_dyn        = zeros(N, 1);             % Dynamic pressure [Pa]
CD_trim      = zeros(N, 1);             % Trimmed drag coefficient [-]
Tr_req       = zeros(N, 1);             % Thrust required [N]
Pr_req       = zeros(N, 1);             % Power required [W]
CL_CD        = zeros(N, 1);             % Aerodynamic efficiency CL/CD [-]
CL32_CD      = zeros(N, 1);             % Endurance factor CL^(3/2)/CD [-]

% Iterative sweep across each angle of attack
for i = 1:N
    % Step 1: Convert angle of attack from degrees to radians
    alpha_rad(i) = deg2rad(alpha_deg(i));
    
    % Step 2: Pitching moment equilibrium (Cm = Cm0 + Cma*alpha + Cme*de = 0)
    de_trim_rad(i) = -(Cm0 + Cma * alpha_rad(i)) / Cme;
    de_trim_deg(i) = rad2deg(de_trim_rad(i));
    
    % Step 3: Trimmed Lift Coefficient (CL = CL0 + CLa*alpha + CLe*de_trim)
    CL_trim(i) = CL0 + CLa * alpha_rad(i) + CLe * de_trim_rad(i);
    
    % Step 4: Steady Level Flight Velocity from Lift Equilibrium (L = W)
    if CL_trim(i) > 0
        V_flight(i) = sqrt((2 * W) / (rho * S * CL_trim(i)));
        q_dyn(i)    = 0.5 * rho * (V_flight(i)^2);
    else
        V_flight(i) = NaN;
        q_dyn(i)    = NaN;
    end
    
    % Step 5: Drag Coefficient from Parabolic Drag Polar (CD = CD0 + k*CL^2)
    CD_trim(i) = CD0 + k * (CL_trim(i)^2);
    
    % Step 6: Thrust Required for Steady Level Flight (Tr = D = 0.5*rho*V^2*S*CD)
    Tr_req(i) = q_dyn(i) * S * CD_trim(i);
    
    % Step 7: Power Required for Steady Level Flight (Pr = Tr * V)
    Pr_req(i) = Tr_req(i) * V_flight(i);
    
    % Phase 2 Metrics: Lift-to-Drag Ratio and Endurance Parameter
    CL_CD(i)   = CL_trim(i) / CD_trim(i);
    CL32_CD(i) = (CL_trim(i)^1.5) / CD_trim(i);
end

%% ========================================================================
%  4. DISPLAY TABULAR RESULTS AND OPTIMAL PERFORMANCE SUMMARY
%  ========================================================================

fprintf('  STEADY LEVEL FLIGHT TRIM & PERFORMANCE TABLE (alpha = 0 to 12 deg)\n');
fprintf('------------------------------------------------------------------------------------------------------------\n');
fprintf('  alpha(deg)  alpha(rad)  de_trim(deg)   CL_trim     CD_trim      V(m/s)      Tr(N)       Pr(W)     CL/CD    CL^1.5/CD\n');
fprintf('------------------------------------------------------------------------------------------------------------\n');
for i = 1:N
    fprintf('    %6.2f      %6.4f       %7.4f     %7.5f     %7.5f     %7.3f    %8.3f    %8.3f   %7.4f    %7.4f\n', ...
        alpha_deg(i), alpha_rad(i), de_trim_deg(i), CL_trim(i), CD_trim(i), ...
        V_flight(i), Tr_req(i), Pr_req(i), CL_CD(i), CL32_CD(i));
end
fprintf('------------------------------------------------------------------------------------------------------------\n\n');

% Identify Discrete Optima from Sweep
[max_LD, idx_LD]       = max(CL_CD);
[min_Tr, idx_Tr]       = min(Tr_req);
[max_Endur, idx_Endur] = max(CL32_CD);
[min_Pr, idx_Pr]       = min(Pr_req);

% Analytical Optima for Parabolic Drag Polar
CL_opt_Tr    = sqrt(CD0 / k);
CD_opt_Tr    = 2 * CD0;
LD_max_ana   = CL_opt_Tr / CD_opt_Tr;
Tr_min_ana   = W / LD_max_ana;
V_Tr_min_ana = sqrt((2 * W) / (rho * S * CL_opt_Tr));

CL_opt_Pr      = sqrt(3 * CD0 / k);
CD_opt_Pr      = 4 * CD0;
Endur_max_ana  = (CL_opt_Pr^1.5) / CD_opt_Pr;
Pr_min_ana     = W * sqrt((2 * W) / (rho * S)) / Endur_max_ana;
V_Pr_min_ana   = sqrt((2 * W) / (rho * S * CL_opt_Pr));

fprintf('======================================================================\n');
fprintf('  KEY PERFORMANCE OPTIMA: DISCRETE SWEEP VS. ANALYTICAL THEORY\n');
fprintf('======================================================================\n');
fprintf('  1. MINIMUM THRUST REQUIRED / MAXIMUM RANGE CONDITION (Max CL/CD):\n');
fprintf('     Discrete Grid Optimum : alpha = %.1f deg, V = %.2f m/s, CL = %.4f, (CL/CD)_max = %.4f, Tr_min = %.3f N\n', ...
    alpha_deg(idx_Tr), V_flight(idx_Tr), CL_trim(idx_Tr), max_LD, min_Tr);
fprintf('     Exact Analytical      : V = %.2f m/s, CL* = %.4f, (CL/CD)_max = %.4f, Tr_min = %.3f N\n\n', ...
    V_Tr_min_ana, CL_opt_Tr, LD_max_ana, Tr_min_ana);

fprintf('  2. MINIMUM POWER REQUIRED / MAXIMUM ENDURANCE CONDITION (Max CL^(3/2)/CD):\n');
fprintf('     Discrete Grid Optimum : alpha = %.1f deg, V = %.2f m/s, CL = %.4f, (CL^1.5/CD)_max = %.4f, Pr_min = %.2f W\n', ...
    alpha_deg(idx_Pr), V_flight(idx_Pr), CL_trim(idx_Pr), max_Endur, min_Pr);
fprintf('     Exact Analytical      : V = %.2f m/s, CL* = %.4f, (CL^1.5/CD)_max = %.4f, Pr_min = %.2f W\n', ...
    V_Pr_min_ana, CL_opt_Pr, Endur_max_ana, Pr_min_ana);
fprintf('======================================================================\n');

%% ========================================================================
%  5. PHASE 3: VISUALIZATION (ALL 8 REQUIRED PLOTS WITH UNITS)
%  ========================================================================
% Note: For velocity-domain plots, index 2:N (alpha = 0.5 to 12 deg) is
% highlighted alongside the full sweep because alpha = 0 deg represents an
% extreme high-speed asymptotic point (V = 92.81 m/s) where lift comes
% purely from positive trailing-edge reflex/elevator deflection.

idx_prac = 2:N;  % Practical flight envelope (alpha >= 0.5 deg, V <= 31.19 m/s)

figure('Name', 'Cropped Delta UAV - Trim & Aerodynamic Coefficients', ...
       'Color', 'w', 'Position', [100, 100, 1200, 800]);

% Plot 1: Trim Angle of Attack vs. Flight Velocity
subplot(2, 2, 1);
plot(V_flight(idx_prac), alpha_deg(idx_prac), 'b-o', 'LineWidth', 1.8, 'MarkerSize', 5);
grid on; xlabel('Flight Velocity, V (m/s)'); ylabel('Trim Angle of Attack, \alpha_{trim} (deg)');
title('1. Trim Angle of Attack vs. Flight Velocity (\alpha \ge 0.5^\circ)');

% Plot 2: Trim Elevator Deflection vs. Angle of Attack
subplot(2, 2, 2);
plot(alpha_deg, de_trim_deg, 'r-s', 'LineWidth', 1.8, 'MarkerSize', 5);
yline(0, 'k--', 'Neutral Elevator (\delta_e = 0^\circ)');
grid on; xlabel('Angle of Attack, \alpha (deg)'); ylabel('Trim Elevator Deflection, \delta_{e,trim} (deg)');
title('2. Trim Elevator Deflection vs. Angle of Attack');

% Plot 3: Trimmed Lift Coefficient vs. Angle of Attack
subplot(2, 2, 3);
plot(alpha_deg, CL_trim, 'Color', [0 0.5 0], 'Marker', '^', 'LineWidth', 1.8, 'MarkerSize', 5);
grid on; xlabel('Angle of Attack, \alpha (deg)'); ylabel('Trimmed Lift Coefficient, C_{L,trim} (-)');
title('3. Trimmed Lift Coefficient vs. Angle of Attack');

% Plot 4: Trimmed Drag Coefficient vs. Angle of Attack
subplot(2, 2, 4);
plot(alpha_deg, CD_trim, 'm-d', 'LineWidth', 1.8, 'MarkerSize', 5);
grid on; xlabel('Angle of Attack, \alpha (deg)'); ylabel('Trimmed Drag Coefficient, C_D (-)');
title('4. Trimmed Drag Coefficient vs. Angle of Attack');

figure('Name', 'Cropped Delta UAV - Propulsion & Efficiency Metrics', ...
       'Color', 'w', 'Position', [150, 150, 1200, 800]);

% Plot 5: Thrust Required vs. Flight Velocity
subplot(2, 2, 1);
plot(V_flight(idx_prac), Tr_req(idx_prac), 'b-o', 'LineWidth', 1.8, 'MarkerSize', 5); hold on;
plot(V_flight(idx_Tr), min_Tr, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
grid on; xlabel('Flight Velocity, V (m/s)'); ylabel('Thrust Required, T_r (N)');
title('5. Thrust Required vs. Flight Velocity (\alpha \ge 0.5^\circ)');
legend('T_r Curve', sprintf('Min T_r = %.2f N (V = %.2f m/s)', min_Tr, V_flight(idx_Tr)), 'Location', 'northwest');

% Plot 6: Power Required vs. Flight Velocity
subplot(2, 2, 2);
plot(V_flight(idx_prac), Pr_req(idx_prac), 'r-s', 'LineWidth', 1.8, 'MarkerSize', 5); hold on;
plot(V_flight(idx_Pr), min_Pr, 'bp', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
grid on; xlabel('Flight Velocity, V (m/s)'); ylabel('Power Required, P_r (W)');
title('6. Power Required vs. Flight Velocity (\alpha \ge 0.5^\circ)');
legend('P_r Curve', sprintf('Min P_r = %.2f W (V = %.2f m/s)', min_Pr, V_flight(idx_Pr)), 'Location', 'northwest');

% Plot 7: Lift-to-Drag Ratio (CL/CD) vs. Angle of Attack
subplot(2, 2, 3);
plot(alpha_deg, CL_CD, 'Color', [0 0.5 0], 'Marker', '^', 'LineWidth', 1.8, 'MarkerSize', 5); hold on;
plot(alpha_deg(idx_LD), max_LD, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
grid on; xlabel('Angle of Attack, \alpha (deg)'); ylabel('Lift-to-Drag Ratio, C_L/C_D (-)');
title('7. Aerodynamic Efficiency (C_L/C_D) vs. Angle of Attack');
legend('C_L/C_D', sprintf('Max C_L/C_D = %.2f (\alpha = %.1f^\circ)', max_LD, alpha_deg(idx_LD)), 'Location', 'southeast');

% Plot 8: Endurance Parameter (CL^(3/2)/CD) vs. Angle of Attack
subplot(2, 2, 4);
plot(alpha_deg, CL32_CD, 'm-d', 'LineWidth', 1.8, 'MarkerSize', 5); hold on;
plot(alpha_deg(idx_Endur), max_Endur, 'bp', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
grid on; xlabel('Angle of Attack, \alpha (deg)'); ylabel('Endurance Parameter, C_L^{3/2}/C_D (-)');
title('8. Endurance Parameter (C_L^{3/2}/C_D) vs. Angle of Attack');
legend('C_L^{3/2}/C_D', sprintf('Max C_L^{3/2}/C_D = %.2f (\alpha = %.1f^\circ)', max_Endur, alpha_deg(idx_Endur)), 'Location', 'southeast');
