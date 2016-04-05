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
l_c1 = (m_1 + 0.5 * m_r1) * l_1 / M_1;
l_c2 = (m_2 + 0.5 * m_r2) * l_1 / M_2;
I_1 = (1/12) * m_r1 * l_1^2 + m_1 * (l_1/2)^2;
I_2 = (1/12) * m_r2 * l_2^2 + m_2 * (l_2/2)^2;

% Joint Torque Limit
tau_max = 20; % [N-m] (Scalar)

% Time
dt = 0.01; % [s] 
t_f = 10; % [s]  

% Initial Conditions
X_0 = [0, 0, 0, 0]; %%%%%%%%%TBD
X_dot_0 = [0, 0]; %%%%%%%%%TBD

% Control Gains (Scalar)
K_p = 10;
K_v = 8;

% Numerical Integration
t = 0:dt:t_f; 
     
X_dot = zeros(length(t),2);
X = zeros(length(t),4);

KE = zeros(length(t));
PE = zeros(length(t));

hold on;
for i = 1:length(t)
    if i == 1
        X_dot(i,:) = X_dot_0;
        X(i,:) = X_0;
    else
    
    % Control torques
    X_d = [0, pi/2];
    tau = transpose(-K_p*(X(i,1:2) - X_d) - K_v * X(i,3:4));
    
    % Apply joint torque limits
    tau(tau > tau_max) = tau_max;
    tau(tau < -tau_max) = -tau_max;
    
    % Dynamic Model
    M = [M_1*l_c1^2+M_2*(l_1+l_c2*cos(X(i,2)))+I_1+I_2, 0;...
        0, M_2*l_c2*2+I_2];
    C = [-2*M_2*l_c2*sin(X(i,2))*(l_1+l_c2*cos(X(i,2))*X(i,3)*X(i,4));...
        M_2*l_c2*sin(X(i,2))*(l_1+l_c2*cos(X(i,2)))];
    G = [0;...
        M_2*g*l_c2*cos(X(i,2))];

    %X = 
    %    [theta1; 
    %     theta2; 
    %     theta1_dot; 
    %     theta2_dot;

    %X_dot = 
    %       [
    %        theta1_double_dot;
    %        theta2_double_dot]
    
    X_dot(i,:) = transpose([inv(M)*(tau - C - G)]); 
    %use the inverse of the inertia matrix M(theta)to solve for [theta1_double_dot; theta2_double_dot]
    
    % Trapezoidal Integration
       %Calculate theta1_dot and theta1_dot at each time increment
    X(i+1,3:4) = X(i,3:4) + X_dot(i,:)*0.01;
    X(i+1,1:2) = X(i,1:2) + 0.5.*(X(i,3:4) + X(i+1,3:4))*0.01;
    
    % Plot Energy
    KE(i) = 0.5 * m_1 * X(i,3)^2 + 0.5 * m_2 * X(i,4)^2;
    PE(i) = m_2 * g * l_2 * cos(X(i,2));
    plot(t(i),KE(i),'*',t(i),PE(i),'.');
    end
    
end
hold off;


% Graphical Simulation
robot.handles = drawRR([X(1,1), X(1,2)],robot);
for i = 2:length(t)
    setRR([X(i,1), X(i,2)],...  %angles
          robot); %robot
    pause(1e-6); % adjustable pause in seconds
end

% Plot Output
figure
plot(t, X(1:length(t),1));
figure
plot(t, X(1:length(t),2));

end

