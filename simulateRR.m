function [  ] = simulateRR(  )
% MECH 498 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function should run a dynamic simulation of the RR
%    robot for this assignment, and then play a video of the resulting
%    robot motion.
%
%
%    ADDITIONAL CODE NEEDED: lots
%    

close all;

% Initialize robot
robot = RRInit();

% Initialize the physical parameters
m_1 = robot.m_1;
m_2 = robot.m_2;
m_r1 = robot.m_r1;
m_r2 = robot.m_r2;
l_1 = robot.l_1;
l_2 = robot.l_2;
M_1 = m_1 + m_r1;
M_2 = m_2 + m_r2;
l_c1 = (m_1 + 0.5 * m_r1) * l1 / M_1;
l_c2 = (m_2 + 0.5 * m_r2) * l1 / M_2;
I_1 = (1/12) * m_r1 * l_1^2 + m_1 * (l_1/2)^2;
I_2 = (1/12) * m_r2 * l_2^2 + m_2 * (l_2/2)^2;

% Joint Torque Limit
tau_max = []; % [N-m] (Scalar)

% Time
dt = []; % [s]
t_f = []; % [s]

% Initial Conditions
X_0 = [];

% Control Gains (Scalar)
K_p = [];
K_v = [];

% Numerical Integration
t = 0:dt:t_f;
X = []; % initialize variable to hold state vector
X_dot = []; % initialize variable to hold state vector derivatives
for i = 1:length(t)
    if i == 1

    else

    end
    
    % Control torques
    tau = [];
    
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;
    
    % Dynamic Model
    M = [];
    C = [];
    G = [];

    X_dot(i,:) = [];
    
    % Trapezoidal Integration
    if i > 1
        
    end
    
    % Plot Energy
    
end

% Graphical Simulation
robot.handles = drawRR([],robot);
for i = 2:length(t)
    setRR([],robot);
    pause(1e-6); % adjustable pause in seconds
end

% Plot Output





end

