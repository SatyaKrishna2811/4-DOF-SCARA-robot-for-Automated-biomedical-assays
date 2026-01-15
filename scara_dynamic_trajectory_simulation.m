clc;
clear all;
close all;

%% ====================================================
%   1. SETUP: 4-DOF SCARA ROBOT MODEL
% ====================================================
a1 = 0.5;   a2 = 0.4;   d1 = 0.5;   d4 = 0.1;

% Define Links
L1 = Link('revolute', 'd', d1, 'a', a1, 'alpha', 0, 'qlim', deg2rad([-160 160]));
L1.m = 4.0; L1.r = [a1/2 0 0]; L1.I = [0.1 0 0; 0 0.1 0; 0 0 0.1]; 

L2 = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0, 'qlim', deg2rad([-150 150]));
L2.m = 3.0; L2.r = [a2/2 0 0]; L2.I = [0.08 0 0; 0 0.08 0; 0 0 0.08];

% Prismatic Joint
L3 = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', 180*pi/180, 'qlim', [0 0.4]);
L3.m = 1.5; L3.r = [0 0 0]; L3.I = [0.05 0 0; 0 0.05 0; 0 0 0.05];

L4 = Link('revolute', 'd', d4, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-360 360]));
L4.m = 0.5; L4.r = [0 0 0]; L4.I = [0.01 0 0; 0 0.01 0; 0 0 0.01];

SCARA = SerialLink([L1 L2 L3 L4], 'name', 'SCARA Dynamic Bot');
SCARA.gravity = [0 0 -9.81];

%% ====================================================
%   2. VISUALIZATION SETUP
% ====================================================
% Initialize figure and plot robot
figure('Color','w'); 
SCARA.plot([0 0 0 0], 'scale', 0.6); 
hold on; grid on;
view(45,30); axis([-1 1 -1 1 0 1.2]);
title('Dynamic Simulation');

% Define Task Points
P_source = [0.6, 0.2, 0.2];
P_target = [0.0, 0.7, 0.2];
safe_z   = 0.5;

% Plot Markers
plot3(P_source(1), P_source(2), P_source(3), 'ro', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Source');
plot3(P_target(1), P_target(2), P_target(3), 'go', 'MarkerSize', 12, 'LineWidth', 3, 'DisplayName', 'Target');
legend show;

%% ====================================================
%   3. MOTION PLANNING (IK SOLVER)
% ====================================================
Cartesian_Waypoints = [
    P_source(1), P_source(2), safe_z;       
    P_source(1), P_source(2), P_source(3);  
    P_source(1), P_source(2), safe_z;       
    P_target(1), P_target(2), safe_z;       
    P_target(1), P_target(2), P_target(3);  
    P_target(1), P_target(2), safe_z        
];

opts = optimoptions('fmincon','Algorithm','sqp','Display','off');
q_guess = [0 0 0 0]; 
Joint_Waypoints = []; 

fprintf('Calculating Inverse Kinematics...\n');

for i = 1:size(Cartesian_Waypoints, 1)
    pos_des = Cartesian_Waypoints(i, :);
    cost_func = @(q) norm( SCARA.fkine(q).t' - pos_des );
    
    LB = [L1.qlim(1), L2.qlim(1), 0, L4.qlim(1)]; 
    UB = [L1.qlim(2), L2.qlim(2), L3.qlim(2), L4.qlim(2)];
    
    [q_sol, val] = fmincon(cost_func, q_guess, [],[],[],[], LB, UB, [], opts);
    
    % Ensure prismatic joint is non-negative
    q_sol(3) = max(0, q_sol(3)); 
    
    Joint_Waypoints = [Joint_Waypoints; q_sol];
    q_guess = q_sol; 
end

%% ====================================================
%   4. DYNAMICS LOOP
% ====================================================
steps_per_segment = 20; 
time_per_segment  = 1.0; 
dt = time_per_segment / steps_per_segment;

fprintf('\nStarting Dynamic Simulation...\n');
fprintf('%-6s | %-10s %-10s %-10s %-10s\n', 'Time', 'Tau1', 'Tau2', 'Force3', 'Tau4');
disp('---------------------------------------------------------');

current_time = 0;
q_prev = [0 0 0 0]; 
all_waypoints = [q_prev; Joint_Waypoints];

for i = 1:size(all_waypoints, 1) - 1
    q_start = all_waypoints(i, :);
    q_end   = all_waypoints(i+1, :);
    
    t_vector = linspace(0, time_per_segment, steps_per_segment);
    [Q, Qd, Qdd] = jtraj(q_start, q_end, t_vector);
    
    for k = 1:steps_per_segment
        q   = Q(k, :);      
        q(3) = max(0, q(3)); % Clamp prismatic joint
        
        qd  = Qd(k, :);     
        qdd = Qdd(k, :);    
        
        tau = SCARA.rne(q, qd, qdd);
        
        SCARA.animate(q); 
        
        current_time = current_time + dt;
        fprintf('%-6.2f | %-10.2f %-10.2f %-10.2f %-10.2f\n', ...
            current_time, tau(1), tau(2), tau(3), tau(4));
            
        drawnow;
    end
end
disp('Simulation Completed.');