%% STEP2_PLAN_TRAJECTORY  Step 2 - Inverse kinematics + joint-space trajectory (UR5)
%  1. Defines a pick-and-place task as Cartesian waypoints
%  2. Solves inverse kinematics for each waypoint (seed-chained for continuity)
%  3. Generates a smooth quintic joint trajectory through them (1 kHz)
%  4. Checks joint speeds and the torques needed (inverse dynamics)
%  5. Animates the arm (optional GIF) and saves the reference for Simulink
%
%  Requires: Robotics System Toolbox, arm_model.mat from step1_model_arm.m
%  Author: Harish Venkatesan M R

clear; clc; close all;
load('arm_model.mat', 'robot', 'eeName', 'q_ready', 'p');
assert(~isempty(robot), 'arm_model.mat has no rigidBodyTree - run step1_model_arm.m with Robotics System Toolbox.');

makeGif  = true;      % writes arm_pick_place.gif for the README (R2022a+)
qdLimit  = pi;        % UR5 max joint speed [rad/s]

%% 1. Cartesian waypoints (relative to the ready pose, constant tool orientation)
T_ready = getTransform(robot, q_ready, eeName);
p0 = T_ready(1:3,4);
R0 = T_ready(1:3,1:3);

offsets = [ 0     0     0;      % start (ready)
            0.10  0.15  0;      % above pick
            0.10  0.15 -0.15;   % pick
            0.10  0.15  0;      % lift
           -0.05 -0.20  0;      % above place
           -0.05 -0.20 -0.15;   % place
           -0.05 -0.20  0;      % lift
            0     0     0 ]';   % back to ready
wpNames = {'start','above pick','pick','lift','above place','place','lift','home'};
nWp = size(offsets, 2);
pWp = p0 + offsets;             % 3 x nWp Cartesian positions

%% 2. Inverse kinematics
ik = inverseKinematics(RigidBodyTree=robot);
ik.SolverParameters.MaxIterations = 500;
weights = [1 1 1 1 1 1];        % [orientation xyz, position xyz]

qWp = zeros(6, nWp);
qGuess = q_ready;
for k = 1:nWp
    Tk = [R0, pWp(:,k); 0 0 0 1];
    [qk, info] = ik(eeName, Tk, weights, qGuess);
    qk = qGuess + angdiff(qGuess, qk);          % nearest equivalent angles -> no 2*pi jumps
    Tchk = getTransform(robot, qk, eeName);
    posErr = norm(Tchk(1:3,4) - Tk(1:3,4));
    rotErr = norm(rotm2axang(Tchk(1:3,1:3)' * R0) .* [0 0 0 1]);
    fprintf('WP %d %-12s pos err = %.1e m | rot err = %.1e rad | %s\n', ...
            k, wpNames{k}, posErr, rotErr, info.Status);
    assert(posErr < 1e-3, 'IK failed at waypoint %d (%s)', k, wpNames{k});
    qWp(:,k) = qk;
    qGuess = qk;
end

%% 3. Quintic joint-space trajectory (stops at each waypoint)
segDur     = [2.0 1.5 1.5 2.5 1.5 1.5 2.0];    % seconds per segment
timePoints = [0, cumsum(segDur)];
dt   = 1e-3;                                   % 1 kHz, matches the controller later
tSamples = 0:dt:timePoints(end);
[q, qd, qdd] = quinticpolytraj(qWp, timePoints, tSamples);
N = numel(tSamples);

qdMax = max(abs(qd), [], 2);
fprintf('\nPeak joint speeds [rad/s]: %s  (limit %.2f)\n', mat2str(qdMax', 3), qdLimit);
assert(all(qdMax < qdLimit), 'Trajectory exceeds joint speed limit - lengthen segDur.');

%% 4. Required torques along the trajectory (inverse dynamics)
idx = 1:10:N;                                  % every 10 ms is plenty for a check
tau = zeros(6, numel(idx));
for i = 1:numel(idx)
    j = idx(i);
    tau(:,i) = inverseDynamics(robot, q(:,j), qd(:,j), qdd(:,j));
end
tauPeak = max(abs(tau), [], 2);
fprintf('Peak required torques [N*m]: %s\n', mat2str(tauPeak', 3));
fprintf('Torque limits         [N*m]: %s\n', mat2str(p.tau_max, 3));
if all(tauPeak < p.tau_max(:))
    fprintf('Trajectory is feasible within torque limits.\n');
else
    warning('Some joints exceed torque limits - slow the trajectory down.');
end

%% 5. Plots
figure('Name','Joint trajectory');
labels = {'q [rad]', 'qd [rad/s]', 'qdd [rad/s^2]'};
data   = {q, qd, qdd};
for s = 1:3
    subplot(3,1,s); plot(tSamples, data{s}', 'LineWidth', 1.2); grid on;
    ylabel(labels{s}); xline(timePoints, ':', 'HandleVisibility','off');
end
xlabel('time [s]'); legend(compose('J%d', 1:6), 'Orientation','horizontal', 'Location','southoutside');
sgtitle('UR5 pick-and-place - quintic joint trajectory');

figure('Name','Required torques');
plot(tSamples(idx), tau', 'LineWidth', 1.2); grid on;
xlabel('time [s]'); ylabel('\tau [N*m]'); legend(compose('J%d', 1:6));
title('Torques required to follow the trajectory (inverse dynamics)');

% End-effector path actually produced by the joint trajectory
eeIdx  = 1:20:N;
eePath = zeros(3, numel(eeIdx));
for i = 1:numel(eeIdx)
    T = getTransform(robot, q(:,eeIdx(i)), eeName);
    eePath(:,i) = T(1:3,4);
end

%% 6. Animation (+ optional GIF)
figure('Name','Pick-and-place animation', 'Color','w');
ax = show(robot, q(:,1), Frames="off", PreservePlot=false);
hold(ax, 'on'); view(ax, 135, 25); axis(ax, 'equal');
plot3(ax, eePath(1,:), eePath(2,:), eePath(3,:), 'b-', 'LineWidth', 1.5);
plot3(ax, pWp(1,:), pWp(2,:), pWp(3,:), 'ro', 'MarkerFaceColor','r');
title(ax, 'UR5 pick-and-place');
gifFile = 'arm_pick_place.gif';
if makeGif && isfile(gifFile), delete(gifFile); end
for j = 1:50:N                                  % ~20 fps playback
    show(robot, q(:,j), Parent=ax, Frames="off", PreservePlot=false, FastUpdate=true);
    drawnow limitrate;
    if makeGif
        try
            exportgraphics(ax, gifFile, Append=true);   % axes only -> no UI warnings
        catch
            makeGif = false;                    % older MATLAB - skip GIF
            warning('GIF export needs R2022a or newer - skipping.');
        end
    end
end

%% 7. Save the reference for Simulink (From Workspace blocks)
qRef   = timeseries(q',   tSamples');
qdRef  = timeseries(qd',  tSamples');
qddRef = timeseries(qdd', tSamples');
Tend   = tSamples(end);
save('trajectory.mat', 'qRef', 'qdRef', 'qddRef', 'qWp', 'pWp', ...
     'timePoints', 'tSamples', 'q', 'qd', 'qdd', 'Tend', 'dt');
fprintf('Saved trajectory.mat - Step 2 complete (%.1f s, %d samples).\n', Tend, N);
