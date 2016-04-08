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
dt = 0.01;
t = t_f / dt;
home_pos_b = via(:,1);
ball_strt = via(:,2);
ball_end = via(:,3);
home_pos_e = via(:,4);
prev_angles = zeros(1,3);
% Initialize variables

path1_x = interp1([0 t/3], [home_pos_b(1); ball_strt(1)], 0:t/3);
path1_y = interp1([0 t/3], [home_pos_b(2); ball_strt(2)], 0:t/3);
path1_z = interp1([0 t/3], [home_pos_b(3); ball_strt(3)], 0:t/3);

path2_x = interp1([t/3 t/3*2], [ball_strt(1); ball_end(1)], t/3:t/3 * 2);
path2_y = interp1([t/3 t/3*2], [ball_strt(2); ball_end(2)], t/3:t/3 * 2);
path2_z = interp1([t/3 t/3*2], [ball_strt(3); ball_end(3)], t/3:t/3 * 2);

path3_x = interp1([t/3*2 t], [ball_end(1); home_pos_e(1)], t/3 * 2:t);
path3_y = interp1([t/3*2 t], [ball_end(2); home_pos_e(2)], t/3 * 2:t);
path3_z = interp1([t/3*2 t], [ball_end(3); home_pos_e(3)], t/3 * 2:t);
% Generate trajectory by interpolating evenly for each target points
% individually for x, y, z coordinates

path1 = [path1_x; path1_y; path1_z];
path2 = [path2_x; path2_y; path2_z];
path3 = [path3_x; path3_y; path3_z];
% Merge the paths 

full_path = horzcat(path1(:,1:1000), path2(:,1:1000), path3(:,1:1001))';
% Concatenate to generate the full trajectory
angle_path = zeros(3,t + 1);
vel_path = zeros(3,t + 1);

for i = 0:1:t
    real_prev = prev_angles;
    [~, joint_angles] = robIK(full_path(i+1,:), prev_angles, rob);
    prev_angles = joint_angles;
    angle_path(:,i+1) = joint_angles';
    vel_path(:,i+1) = (joint_angles' - real_prev') ./ dt; % should it be 1?
end

trajectory(1,:) = 0:dt:t_f; %Time
trajectory(2:4,:) = angle_path;  %Joint angles
trajectory(5:7,:) = vel_path; %Joint velocities

end

