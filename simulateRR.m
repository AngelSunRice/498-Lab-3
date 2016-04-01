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
g = robot.g;

M_1 = m_1 + m_r1;
M_2 = m_2 + m_r2;
l_c1 = (m_1 + 0.5 * m_r1) * l1 / M_1;
l_c2 = (m_2 + 0.5 * m_r2) * l1 / M_2;
I_1 = (1/12) * m_r1 * l_1^2 + m_1 * (l_1/2)^2;
I_2 = (1/12) * m_r2 * l_2^2 + m_2 * (l_2/2)^2;

% Joint Torque Limit
tau_max = []; % [N-m] (Scalar)

% Time
dt = 0.01; % [s] 
t_f = 10; % [s]  

% Initial Conditions
X_0 = [0;...
       0]; %%%%%%%%%TBD

% Control Gains (Scalar)
K_p = [];
K_v = [];

% Numerical Integration
t = 0:dt:t_f;
X = X_0; % initialize variable to hold state vector 
X_dot = [0;...
         0]; % initialize variable to hold state vector derivatives  %%%%%%%%%TBD
for i = 1:length(t)
    if i == 1

    else

    end
    
    % Control torques
    tau = [1;...
           2];
    
    % Apply joint torque limits
    tau(tau>tau_max) = tau_max;
    tau(tau<-tau_max) = -tau_max;
    
    % Dynamic Model
    M = [M_1*l_c1^2+M_2*(l_1+l_c2*cos(X(2)))+I_1+I_2, 0;...
        M_2*l_c2*sin(X(2))*(l_1+l_c2*cos(X(2))), M_2*l_c2*2+I_2];
    C = [-2*M_2*l_c2*sin(X(2))*(l_1+l_c2*cos(X(2))*X(3)*X(4));...
        0];
    G = [0;...
        M_2*g*l_c2*cos(X(2))];

    X_dot(i,:) = [X(3);...
                  X(4);...
                  M\(tau-C-G)];
    
    % Trapezoidal Integration
    if i > 1
        
    end
    
    % Plot Energy
    
end

% Graphical Simulation
robot.handles = drawRR([],robot);
for i = 2:length(t)
    setRR([X(1), X(2)],...  %angles
          robot); %robot
    pause(1e-6); % adjustable pause in seconds
end

% Plot Output





end

