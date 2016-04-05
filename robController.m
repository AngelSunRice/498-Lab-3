% yg18, as99

function [tau] = robController(trajectory, Theta, Theta_dot, t, rob)

% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - This function sends command torques in the vector tau to move the
%    robot. The controller determines tau based on the desired position and
%    velocity and the actual position and velocity. Desired position and
%    velocity are obtained from trajectory and the current time t. The
%    actua position is Theta, and the actual velocity is Theta_dot.
%    Calculate the torques needed to add gravity compensation using the
%    robot parameters from the structure rob.

% Robot Parameters from rob
g = rob.parameters.g;
...
%keyboard
% Gravity Compensation Vector
G = []; %[3x1] vector

% Trajectory interpolation (DO NOT CHANGE)
Theta_ref = zeros(3,1);
Theta_dot_ref = zeros(3,1);

for i = 1:3
    Theta_ref(i) = interp1(trajectory(1,:),trajectory(i+1,:),t);    
    Theta_dot_ref(i) = interp1(trajectory(1,:),trajectory(i+4,:),t);
end
%keyboard
% Gravity Compensation Control

K_p = [505 755 755]; % Proportional gain matrix containing gains K_p1 to K_p3
K_v = [20 20 20]; % Derivative gain matrix containing gains K_v1 to K_v3

% control input (torque)
tau = - K_p * (Theta - Theta_ref) - K_v * (Theta_dot - Theta_dot_ref); %+ G;

end

