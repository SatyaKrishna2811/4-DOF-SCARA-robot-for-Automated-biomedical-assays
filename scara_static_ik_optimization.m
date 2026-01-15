clc;
clear all;
close all;

%% ====================================================
%   1. SETUP: 4-DOF SCARA ROBOT MODEL WITH DYNAMICS
% ====================================================

% --- 1.1 KINEMATIC PARAMETERS (DH Parameters) ---
% These define the geometry of the robot arm.
a1 = 0.5;   % [m] Length of Link 1 (Shoulder Arm)
a2 = 0.4;   % [m] Length of Link 2 (Elbow Arm)
d1 = 0.5;   % [m] Base Height (Distance from ground to shoulder)
d4 = 0.1;   % [m] Tool Length (Vertical offset of   end-effector)

% --- 1.2 LINK DEFINITIONS WITH DYNAMICS (Mass & Inertia) ---
% We define each link with its kinematic type and physical properties.

% L1: Shoulder Link (Rotational)
% 'd': Offset along Z, 'a': Length along X, 'alpha': Twist
L1 = Link('revolute', 'd', d1, 'a', a1, 'alpha', 0, 'qlim', deg2rad([-160 160]));
L1.m = 4.0;                         % [kg] Mass of Link 1
L1.r = [a1/2 0 0];                  % [m]  Center of Mass position (relative to joint)
L1.I = [0.1 0 0; 0 0.1 0; 0 0 0.1]; % [kg·m²] Inertia Tensor [Ixx Iyy Izz]

% L2: Elbow Link (Rotational)
L2 = Link('revolute', 'd', 0,  'a', a2, 'alpha', 0, 'qlim', deg2rad([-150 150]));
L2.m = 3.0;                         % [kg] Mass of Link 2
L2.r = [a2/2 0 0];                  % [m]  Center of Mass position
L2.I = [0.08 0 0; 0 0.08 0; 0 0 0.08]; % [kg·m²] Inertia Tensor

% L3: Linear Vertical Link (Prismatic/Sliding)
% Note: 'prismatic' joint type. 'alpha' = 180 flips Z-axis for downward motion.
L3 = Link('prismatic', 'theta', 0, 'a', 0, 'alpha', 180*pi/180, 'qlim', [0 0.4]);
L3.m = 1.5;                         % [kg] Mass of Link 3 (The sliding shaft)
L3.r = [0 0 0];                     % [m]  Center of Mass (centered)
L3.I = [0.05 0 0; 0 0.05 0; 0 0 0.05]; % [kg·m²] Inertia Tensor

% L4: Wrist Roll Link (Rotational - Rotates Z)
L4 = Link('revolute', 'd', d4, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-360 360]));
L4.m = 0.5;                         % [kg] Mass of Link 4 (The End-Effector)
L4.r = [0 0 0];                     % [m]  Center of Mass
L4.I = [0.01 0 0; 0 0.01 0; 0 0 0.01]; % [kg·m²] Inertia Tensor

% --- 1.3 ROBOT ASSEMBLY ---
% Combine links into a SerialLink object and define world gravity.
SCARA = SerialLink([L1 L2 L3 L4], 'name', 'SCARA 4-DOF Optimization Bot');
SCARA.gravity = [0 0 -9.81];        % [m/s²] Gravity vector (Standard Earth Gravity)

%% ====================================================
%   2. WAYPOINTS & VISUALIZATION
% ====================================================

% --- 2.1 TASK COORDINATES ---
% Define the Pick and Place locations in 3D space [X, Y, Z].
P_source = [0.6, 0.2, 0.2];        % [m] Source Location (Pick)
P_target = [0.0, 0.7, 0.2];        % [m] Target Location (Place)
safe_z   = 0.5;                     % [m] Safe Clearance Height for travel

% --- 2.2 WAYPOINT MATRIX ---
% Ordered list of targets the robot must reach sequentially.
% Format: [X, Y, Z]
Waypoints = [
    P_source(1), P_source(2), safe_z;       % 1. Move to Safe Height above Source
    P_source(1), P_source(2), P_source(3);  % 2. Dive Down to Source (Pick)
    P_source(1), P_source(2), safe_z;       % 3. Lift Back to Safe Height
    P_target(1), P_target(2), safe_z;       % 4. Move to Safe Height above Target
    P_target(1), P_target(2), P_target(3);  % 5. Dive Down to Target (Place)
    P_target(1), P_target(2), safe_z        % 6. Retract to Safe Height
];

% --- 2.3 PLOT SETUP ---
figure('Color','w');
hold on; grid on;
axis([-1 1 -1 1 0 1.2]);            % Set 3D axes limits
view(45,30);                        % Set viewing angle
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('SCARA 4-DOF: Optimization IK + Dynamics Analysis');

% Draw Source (Red) and Target (Green) Markers
plot3(P_source(1),P_source(2),P_source(3), 'ro', 'MarkerSize', 10, 'LineWidth', 3, 'DisplayName', 'Source');
plot3(P_target(1),P_target(2),P_target(3), 'go', 'MarkerSize', 10, 'LineWidth', 3, 'DisplayName', 'Target');
legend show;
    
% Plot Robot in Initial Configuration (Zero position)
SCARA.plot([0 0 0 0]);

% --- 2.4 OPTIMIZATION SETUP ---
% Options for fmincon (Sequential Quadratic Programming)
opts = optimoptions('fmincon','Algorithm','sqp','Display','off','StepTolerance',1e-4);
q_sol = [0 0 0 0];                  % Initial guess for joint angles [rad]

%% ====================================================
%   3. TRAJECTORY LOOP (OPTIMIZATION + DYNAMICS)
% ====================================================

steps = 18;                         % Number of interpolation steps per segment
disp('Starting Trajectory Optimization with Dynamics...');
disp('---------------------------------------------------------------------------------------------------------');
% Print Table Headers
fprintf('%-6s %-8s %-8s %-8s %-8s | %-10s %-10s %-10s\n', ...
    'Step', 'X_err', 'Y_err', 'Z_err', 'Cost', 'Torque1(Nm)', 'Torque2(Nm)', 'Force3(N)');
disp('---------------------------------------------------------------------------------------------------------');

step_counter = 0;                   % Initialize global step counter

% Loop through each segment defined in Waypoints
for i = 1:size(Waypoints,1)-1
    P_start = Waypoints(i,:);       % [m] Start point of current segment
    P_end   = Waypoints(i+1,:);     % [m] End point of current segment
    
    % Interpolate between Start and End points
    for t = linspace(0, 1, steps)
        step_counter = step_counter + 1;
        
        % 1. Desired Cartesian Position for this step
        pos_des = (1-t)*P_start + t*P_end;
        
        % 2. Cost Function: Minimize Euclidean distance between End-Effector and Target
        cost_function = @(q) norm( SCARA.fkine(q).t' - pos_des );
        
        % 3. Joint Limits Constraints (Lower Bound & Upper Bound)
        LB = [L1.qlim(1), L2.qlim(1), L3.qlim(1), L4.qlim(1)];
        UB = [L1.qlim(2), L2.qlim(2), L3.qlim(2), L4.qlim(2)];
        
        % 4. Solve Inverse Kinematics (Optimization)
        % finds q_sol that minimizes cost_function
        [q_sol, final_cost] = fmincon(cost_function, q_sol, [], [], [], [], LB, UB, [], opts);
        
        % 5. DYNAMICS CALCULATION
        % Calculate Joint Torques/Forces required to hold position against Gravity
        tau = SCARA.gravload(q_sol); % Returns [Nm, Nm, N, Nm]
        
        % 6. Update Robot Visual
        SCARA.animate(q_sol);
        
        % 7. Log Data to Command Window
        current_pos = SCARA.fkine(q_sol).t'; % Actual Position from FK
        err = pos_des - current_pos;         % Position Error
        
        fprintf('%-6d %-8.3f %-8.3f %-8.3f %-8.4f | %-10.2f %-10.2f %-10.2f\n', ...
            step_counter, err(1), err(2), err(3), final_cost, tau(1), tau(2), tau(3));
        
        drawnow; % Force MATLAB to draw the plot update
    end
end
disp('Simulation Completed.');