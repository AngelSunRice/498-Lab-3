function [ trajectory ] = createRobTrajectory( via, rob )

% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 3
% Solutions by Craig McDonald
%
%    DESCRIPTION - Generate a joint position and velocity trajectory to be
%    used by the controller.
%
%    The input via is a 3x4 matrix containing via points for the end
%    effector. Each column contains a point in 3D space. The trajectory
%    should move sequentially through each via point. The best approach is
%    to simply move in a straight line in cartesian space between the
%    points. Use robIK() to find the corresponding trajectory in joint
%    space.
%
%    The output trajectory is a 7xn matrix. The first row will be
%    equally-spaced time stamps starting at time zero and finishing at time
%    t_f given to you below. Rows 2 through 4 contain joint angles,
%    and rows 5 7 should contain joint velocities.

t_f = 30; % final time (do not change) [s]

home_pos_b = via(:,1)';
ball_strt = via(:,2)';
ball_end = via(:,3)';
home_pos_e = via(:,4)';
prev_angles = zeros(1,6);

path1 = interp1([1 t_f/3], [home_pos_b; ball_strt], 1:t_f / 3);
path2 = interp1([t_f/3 + 1 t_f/3*2], [ball_strt; ball_end], t_f/3 + 1:t_f/3 * 2);
path3 = interp1([t_f/3*2 + 1 t_f], [ball_end; home_pos_e], t_f/3*2 + 1:t_f);
full_path = [path1; path2; path3];
angle_path = zeros(3,t_f);
vel_path = zeros(3,t_f);

for i = 1:1:t_f
    [~, joint_angles] = robIK(full_path(i,:), prev_angles, rob);
    prev_angles = [joint_angles 0 0 0];
    angle_path(:,i) = joint_angles(1:3)';
    vel_path(:, i) = angle_path(:,i).* angle_path(:,i);
end

trajectory(1,:) = 1:t_f; %Time
trajectory(2:4,:) = angle_path; %Joint angles
trajectory(5:7,:) = vel_path; %Joint velocities

end

