% Rocker-Bogie Bounded 30cm Length Optimization
clear; clc; close all;

%% 1. System Parameters
L_max = 0.30;           % Absolute max length including wheels (m)
x_cg = L_max / 2;       % CoM assumed at geometric center (0.15 m)
mass_total = 15;        % Total rover mass (kg)
g = 9.81;               % Gravity (m/s^2)
W = mass_total * g;     
h = 0.03;               % Obstacle height (m) - 3 cm
V_rover = 0.01;         % Target forward velocity (m/s)

%% 2. Sweep Ranges (Front/Mid Radius vs Rear Radius)
% Sweeping radii from 1.6cm (just enough to clear 3cm step) to 8cm
Rf_vec = linspace(0.04, 0.04, 2000); 
Rr_vec = linspace(0.016, 0.08, 2000);

[R_f, R_r] = meshgrid(Rf_vec, Rr_vec);

%% 3. Wheel Centers & Pivot for Load Equalization
% Front wheel center
x_f = R_f; 

% Rear wheel center (bounded by max length)
x_r = L_max - R_r; 

% To perfectly equalize load (W/3 per wheel), the sum of moments demands:
% x_cg = (x_r + 2*x_p) / 3  =>  x_p = (3*x_cg - x_r) / 2
x_p = (3 .* x_cg - x_r) ./ 2;

% For rocker to split load evenly between front and mid, pivot must be centered
% x_p = (x_f + x_m) / 2  =>  x_m = 2*x_p - x_f
x_m = 2 .* x_p - x_f;

%% 4. Physical Constraints (Collision Detection)
clearance = 0.01; % 1cm minimum gap between wheels

% Constraint 1: Front and Mid wheels don't collide
c1 = (x_m - x_f) >= (2 .* R_f + clearance);

% Constraint 2: Mid and Rear wheels don't collide
c2 = (x_r - x_m) >= (R_f + R_r + clearance);

% Constraint 3: Obstacle clearance (Radius must be > h/2, practically > h)
c3 = R_f > h;

valid_mask = c1 & c2 & c3;

%% 5. Power Calculations (Torque * RPM)
% Load per wheel in neutral position is W/3
N_wheel = W / 3;

% Power to lift over 3cm step: P = N * V * sqrt(2Rh - h^2) / (R-h)
P_req = (N_wheel .* V_rover .* sqrt(max(0, 2.*R_f.*h - h.^2))) ./ (R_f - h);
P_req(~valid_mask) = inf; % Reject invalid geometries

%% 6. Find Optimum
[min_power, idx] = min(P_req(:));
[min_Rr_idx, min_Rf_idx] = ind2sub(size(P_req), idx);

opt_Rf = Rf_vec(min_Rf_idx);
opt_Rr = Rr_vec(min_Rr_idx);

fprintf('--- Optimized Geometry for 30cm Hard Limit ---\n');
if isinf(min_power)
    fprintf('No valid configuration found! Clearances are too tight.\n');
else
    fprintf('Front/Mid Wheel Radius: %.2f cm\n', opt_Rf * 100);
    fprintf('Rear Wheel Radius: %.2f cm\n', opt_Rr * 100);
    fprintf('Front Wheel Center (x_f): %.2f cm\n', opt_Rf * 100);
    fprintf('Middle Wheel Center (x_m): %.2f cm\n', (2* ((3*x_cg - (L_max - opt_Rr))/2) - opt_Rf) * 100);
    fprintf('Rear Wheel Center (x_r): %.2f cm\n', (L_max - opt_Rr) * 100);
    fprintf('Rocker Pivot Location (x_p): %.2f cm\n', ((3*x_cg - (L_max - opt_Rr))/2) * 100);
    fprintf('Peak Power to Clear 3cm Obstacle: %.2f W per powered wheel\n', min_power);
end

%% 7. Visualization
%{
figure('Color', 'w');
surf(R_f.*100, R_r.*100, P_req, 'EdgeColor', 'none');
hold on;
plot3(opt_Rf*100, opt_Rr*100, min_power, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
colormap parula; colorbar; caxis([0 5]);
title('Peak Power per Powered Wheel');
xlabel('Front/Mid Radius (cm)'); ylabel('Rear Radius (cm)'); zlabel('Power (W)');
view(-45, 45); grid on;\
%}