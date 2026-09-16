%{
Title: Differental_Gear_Torque
Author: Harrison Good
Purpose: Calculates the amount of torque required for a two axled rover to
climb over a rigid obstacle
%}

clear;
clc;
close all;

%% Standard variables

g = 9.81; % [m/s^2]
N = 1000; % number of calculation points

%% Rover details

D = .1; % [m], diameter of wheels
cg = .15; % [m], location of center of gravity from front of rover
h = .15; % [m], height of center of gravity
obstacle_height = .03; % [m], height of obstacle
w = 15; % [kg], mass of rover
mu = .7; % [], coefficient of friction between wheels and ground
efficiency = .8; % powertrain efficiency

%% Calculated Constants

L = .2; % [m], wheelbase
Cg = cg-.5*D; % [m], location of center of gravity from front of wheels
d = linspace(obstacle_height,1e-6,N); % vector of remaining obstacle height
R = .5*D; % [m], radius of wheels

phi = atan((obstacle_height-d)/L); % [rad], angle of the rover

% Geometry of front wheel contact with obstacle
a = sqrt(2*R.*d-d.^2); % [m], horizontal distance from wheel center to obstacle corner
b = R-d; % [m], vertical distance from obstacle corner to wheel center
theta = atan2(a,b); % [rad], angle of contact force from vertical

%% Calculations of Force Specs

W = w*g; % [N], total weight

% Initialize force arrays
FN = zeros(1,N); % [N], normal force at front wheel
F_front = zeros(1,N); % [N], tangential force at front wheel
F_rear = zeros(1,N); % [N], driving force at rear wheel

wF = zeros(1,N); % [N], vertical force supported by front wheel
wR = zeros(1,N); % [N], vertical force supported by rear wheel

T_front = zeros(1,N); % [N*m], front wheel torque
T_rear = zeros(1,N); % [N*m], rear wheel torque
T_total = zeros(1,N); % [N*m], total wheel torque
T_motor = zeros(1,N); % [N*m], motor torque

Ftraction_front = zeros(1,N); % [N], front traction limit
Ftraction_rear = zeros(1,N); % [N], rear traction limit

%% Calculations of Force Specs

for i = 1:N

    % Current geometry
    A = L*cos(phi(i)) + a(i); % [m], x location of obstacle contact from rear axle
    B = L*sin(phi(i)) - b(i); % [m], y location of obstacle contact from rear axle

    % CG location relative to rear axle
    xCG = Cg*cos(phi(i))-h*sin(phi(i)); % [m]
    yCG = Cg*sin(phi(i))+h*cos(phi(i)); % [m]

    % Unit vectors at front wheel/obstacle contact
    nx = -a(i)/R; % horizontal component of normal force
    ny = b(i)/R; % vertical component of normal force

    tx = b(i)/R; % horizontal component of tangential force
    ty = a(i)/R; % vertical component of tangential force

    % Moment equilibrium about rear axle
    %
    % Nf = normal force at obstacle
    % Ft = tangential force at obstacle
    %
    % Nf*(A*ny-B*nx) + Ft*(A*ty-B*tx) - W*xCG = 0

    Cn = A*ny-B*nx;
    Ct = A*ty-B*tx;

    % Search for minimum total driving force
    Ft_test = linspace(0,mu*W,1000);

    % Normal force at obstacle required for moment equilibrium
    N_test = (W*xCG-Ft_test*Ct)/Cn;

    % Rear driving force from horizontal force equilibrium
    Fr_test = N_test*(-nx)-Ft_test*tx;

    % Rear normal force from vertical force equilibrium
    Nr_test = W-N_test*ny-Ft_test*ty;

    % Friction constraints
    front_traction = Ft_test <= mu*N_test;
    rear_traction = Fr_test <= mu*Nr_test;

    % Contact force must remain compressive
    front_contact = N_test >= 0;
    rear_contact = Nr_test >= 0;

    % Driving forces must be positive
    front_drive = Ft_test >= 0;
    rear_drive = Fr_test >= 0;

    % Determine feasible solutions
    feasible = front_traction & rear_traction & ...
               front_contact & rear_contact & ...
               front_drive & rear_drive;

    if any(feasible)

        % Total driving force
        F_total_test = Ft_test + Fr_test;

        % Set infeasible solutions to infinity
        F_total_test(~feasible) = inf;

        % Find minimum total driving force
        [~,index] = min(F_total_test);

        % Store minimum forces
        F_front(i) = Ft_test(index);
        F_rear(i) = Fr_test(index);
        FN(i) = N_test(index);

        % Vertical force supported by front and rear
        wF(i) = FN(i)*ny + F_front(i)*ty;
        wR(i) = Nr_test(index);

    else

        % No feasible solution
        F_front(i) = NaN;
        F_rear(i) = NaN;
        FN(i) = NaN;
        wF(i) = NaN;
        wR(i) = NaN;

    end

    % Traction limits
    Ftraction_front(i) = mu*FN(i);
    Ftraction_rear(i) = mu*wR(i);

    % Torque required at front and rear wheels
    T_front(i) = F_front(i)*R;
    T_rear(i) = F_rear(i)*R;

    % Total wheel torque
    T_total(i) = T_front(i)+T_rear(i);

    % Motor torque accounting for powertrain efficiency
    T_motor(i) = T_total(i)/efficiency;

end

x = obstacle_height-d;

%% Plot Torque

figure(1)
hold on
plot(x,T_total, 'r')
plot(x,T_front, 'b')
plot(x,T_rear, 'y')
plot(x,T_motor, 'k')
xlabel('Distance climbed [m]')
ylabel('Torque [N*m]')
legend('Total Wheel Torque','Front Wheel Torque','Rear Wheel Torque','Motor Torque')
grid on

%% Plot Forces

figure(2)
hold on
plot(x,F_front, 'b')
plot(x,F_rear, 'y')
plot(x,FN, 'r')
plot(x,wR, 'k')
xlabel('Distance climbed [m]')
ylabel('Force [N]')
legend('Front Driving Force','Rear Driving Force','Front Normal Force','Rear Normal Force')
grid on

%% Plot Torque Distribution

figure(3)
hold on
plot(x,T_front./T_total*100, 'b')
plot(x,T_rear./T_total*100, 'y')
xlabel('Distance climbed [m]')
ylabel('Torque Distribution [%]')
legend('Front Torque','Rear Torque')
ylim([0 100])
grid on