%% Rover Obstacle-Climbing Torque Model
% 4-wheel rover
% Front and rear axles separated by L
% Front wheels climb a rigid obstacle
%
% Based on the moment/force relationships in:
% Thomas & Vantsevich (2010), Wheel-terrain-obstacle interaction
% in vehicle mobility analysis

clear;
clc;
close all;

%% =========================
%  ROVER PARAMETERS
%  ==========================

m = 15;                 % Rover mass [kg]
g = 9.81;               % Gravity [m/s^2]

R = 0.05;               % Wheel radius [m]
L = 0.20;               % Distance between axles [m]
h = 0.03;               % Obstacle height [m]

% Center of gravity location
xCG = 0.10;              % CG distance from rear axle [m]
zCG = 0.10;              % CG height above axle centerline [m]

% Tire-ground friction coefficient
mu = 0.6;

% Rolling resistance coefficient
f = 0.0;

%% =========================
%  SIMULATION RANGE
%  ==========================

% Vertical height climbed by FRONT axle
% 0 = beginning of climb
% h = front axle reaches top of obstacle

N = 500;

y = linspace(0,h,N);

%% =========================
%  PREALLOCATE
%  ==========================

theta = zeros(1,N);

W_front = zeros(1,N);
W_rear  = zeros(1,N);

x_corner = zeros(1,N);
y_corner = zeros(1,N);

F_frame_max = zeros(1,N);

T_front = zeros(1,N);
T_rear  = zeros(1,N);
T_total = zeros(1,N);

%% =========================
%  MAIN SIMULATION
%  ==========================

for i = 1:N

    %% Rover pitch angle
    %
    % Front axle rises by y while rear axle stays on ground.
    
    theta(i) = asin(y(i)/L);

    %% Effective axle spacing projected horizontally
    
    L_horizontal = L*cos(theta(i));

    %% CG horizontal position relative to rear axle
    %
    % As rover pitches, CG moves relative to the axle contact points.
    
    xCG_horizontal = xCG*cos(theta(i)) - zCG*sin(theta(i));

    %% Front/rear normal loads
    %
    % Static moment balance about rear axle:
    %
    % N_front * L_horizontal = W * xCG_horizontal
    
    W_total = m*g;

    W_front(i) = W_total * xCG_horizontal/L_horizontal;

    W_rear(i) = W_total - W_front(i);

    %% =========================
    %  OBSTACLE GEOMETRY
    %  ==========================
    
    % Vertical distance between front wheel center
    % and obstacle corner
    
    vertical_distance = R - h + y(i);

    % Prevent numerical problems if geometry becomes invalid
    vertical_distance = min(vertical_distance,R);

    % Horizontal distance from obstacle corner
    % to wheel center
    
    x_corner(i) = sqrt(R^2 - vertical_distance^2);

    y_corner(i) = vertical_distance;

    %% =========================
    %  REAR-WHEEL TRACTION
    %  ==========================
    
    % Maximum force the rear axle can provide
    %
    % Based on:
    % F_frame,max = (mu - f)*W_rear
    
    F_frame_max(i) = max(0,(mu-f)*W_rear(i));

    %% =========================
    %  FRONT-WHEEL TORQUE
    %  ==========================
    
    % Front wheel must overcome the gravitational moment
    % about the obstacle corner.
    %
    % T_front = W_front*x_corner
    %           - F_frame*y_corner
    %
    % For the moment, assume rear axle provides its
    % maximum useful traction.
    
    T_front(i) = W_front(i)*x_corner(i) - F_frame_max(i)*y_corner(i);

    % Torque cannot be negative
    T_front(i) = max(0,T_front(i));

    %% =========================
    %  REAR-WHEEL TORQUE
    %  ==========================
    
    % Torque required at rear axle to generate
    % the frame force.
    
    T_rear(i) = F_frame_max(i)*R;

    %% =========================
    %  TOTAL TORQUE
    %  ==========================
    
    T_total(i) = T_front(i) + T_rear(i);

end

%% =========================
%  RESULTS
%  ==========================

fprintf('\n====================================\n');
fprintf('ROVER OBSTACLE CLIMB RESULTS\n');
fprintf('====================================\n');

[maxFront,idxF] = max(T_front);
[maxRear,idxR]  = max(T_rear);
[maxTotal,idxT] = max(T_total);

fprintf('\nMaximum FRONT axle torque:\n');
fprintf('  %.3f N*m\n',maxFront);
fprintf('  at %.1f mm climb\n',y(idxF)*1000);

fprintf('\nMaximum REAR axle torque:\n');
fprintf('  %.3f N*m\n',maxRear);
fprintf('  at %.1f mm climb\n',y(idxR)*1000);

fprintf('\nMaximum TOTAL torque:\n');
fprintf('  %.3f N*m\n',maxTotal);
fprintf('  at %.1f mm climb\n',y(idxT)*1000);

fprintf('\n====================================\n');

%% =========================
%  PLOTS
%  ==========================

figure;

plot(y*1000,T_front,'LineWidth',2);
hold on;
plot(y*1000,T_rear,'LineWidth',2);
plot(y*1000,T_total,'LineWidth',2);

grid on;

xlabel('Front axle height [mm]');
ylabel('Required torque [N*m]');

title('Rover Torque Requirement During Obstacle Climb');

legend('Front axle','Rear axle','Total','Location','best');

%% =========================
%  AXLE LOAD PLOT
%  ==========================

figure;

plot(y*1000,W_front,'LineWidth',2);
hold on;
plot(y*1000,W_rear,'LineWidth',2);

grid on;

xlabel('Front axle height [mm]');
ylabel('Normal load [N]');

title('Front and Rear Axle Loading During Climb');

legend('Front axle','Rear axle','Location','best');

%% =========================
%  FRAME FORCE PLOT
%  ==========================

figure;

plot(y*1000,F_frame_max,'LineWidth',2);

grid on;

xlabel('Front axle height [mm]');
ylabel('Rear axle force [N]');

title('Maximum Rear-Wheel Frame Force');