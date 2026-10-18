%% STEP1_MODEL_ARM  Step 1 - Model a 6-DOF arm (UR5) in MATLAB/Simulink
%  Builds the arm model, verifies forward kinematics, visualises it, and saves
%  everything to arm_model.mat for the later steps (IK, trajectory, control).
%
%  Works with or without Robotics System Toolbox:
%   - WITH the toolbox  -> loads the UR5 as a rigidBodyTree (kinematics + dynamics)
%   - WITHOUT it        -> uses the hand-coded DH model in dh_fk.m
%
%  Author: Harish Venkatesan M R

clear; clc; close all;

p = ur5_params();
hasRST = license('test','Robotics_System_Toolbox') && exist('loadrobot','file') == 2;
hasSM  = license('test','Simscape_Multibody')      && exist('smimport','file')  == 2;
fprintf('Robotics System Toolbox: %d | Simscape Multibody: %d\n', hasRST, hasSM);

%% 1. Test configurations
q_home  = zeros(6,1);
q_ready = [0; -pi/2; pi/2; -pi/2; -pi/2; 0];   % typical "ready" pose
q_test  = [0.3; -1.2; 1.0; -0.8; -1.4; 0.5];   % arbitrary pose

%% 2. DH forward kinematics (always available)
configs = {q_home, q_ready, q_test};
names   = {'home','ready','test'};
for k = 1:numel(configs)
    T = dh_fk(configs{k}, p);
    fprintf('DH FK [%5s]  EE position = [% .4f % .4f % .4f] m\n', ...
            names{k}, T(1:3,4));
end

% Sanity check: at q = 0 the UR5 end-effector is fully stretched along -x
T0 = dh_fk(q_home, p);
expected = [p.a(2)+p.a(3); -(p.d(4)+p.d(6)); p.d(1)-p.d(5)];
assert(norm(T0(1:3,4) - expected) < 1e-9, 'DH FK sanity check failed');
fprintf('DH sanity check passed (home pose matches analytic value).\n');

%% 3. Toolbox model (if available)
robot = [];
if hasRST
    robot = loadrobot("universalUR5", DataFormat="column", Gravity=[0 0 -9.81]);
    fprintf('Loaded rigidBodyTree "universalUR5" with %d bodies.\n', robot.NumBodies);
    eeName = robot.BodyNames{end};

    % Dynamics check - these are what the computed-torque controller will use
    M = massMatrix(robot, q_ready);
    G = gravityTorque(robot, q_ready);
    fprintf('Mass matrix at ready pose is %dx%d, symmetric: %d, pos-def: %d\n', ...
        size(M,1), size(M,2), norm(M-M','fro') < 1e-9, all(eig(M) > 0));
    fprintf('Gravity torques at ready pose [N*m]: %s\n', mat2str(G', 3));

    figure('Name','UR5 rigidBodyTree');
    show(robot, q_test, Frames="off", PreservePlot=false);
    title('UR5 (rigidBodyTree) - test pose'); axis equal; view(135,25);
else
    eeName = 'tool0';
    fprintf('Toolbox not found - continuing with the DH model only.\n');
end

%% 4. Stick-figure plot from the DH model (works for everyone)
figure('Name','UR5 DH model');
hold on; grid on; axis equal; view(135,25);
cols = lines(numel(configs));
for k = 1:numel(configs)
    [~, Tall] = dh_fk(configs{k}, p);
    P = squeeze(Tall(1:3,4,:));
    plot3(P(1,:), P(2,:), P(3,:), '-o', 'LineWidth', 2.5, ...
          'Color', cols(k,:), 'MarkerFaceColor', cols(k,:));
    drawFrame(Tall(:,:,end), 0.08);
end
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend(names, 'Location','best');
title('UR5 - DH forward kinematics');

%% 5. Workspace sampling (nice figure for the README)
N = 5000;
pts = zeros(3,N);
for i = 1:N
    qr = (rand(6,1)*2 - 1) * pi;
    T = dh_fk(qr, p);
    pts(:,i) = T(1:3,4);
end
figure('Name','Reachable workspace');
scatter3(pts(1,:), pts(2,:), pts(3,:), 3, pts(3,:), 'filled');
axis equal; grid on; view(135,25); colorbar;
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title(sprintf('UR5 reachable workspace (%d random samples)', N));
fprintf('Max reach from samples: %.3f m\n', max(vecnorm(pts)));

%% 6. Optional: Simscape Multibody model for 3D physics in Simulink
if hasRST && hasSM
    urdfPath = fullfile(tempdir, 'ur5.urdf');
    exportrobot(robot, OutputFileName=urdfPath);
    smimport(urdfPath, ModelName="ur5_plant");     % creates ur5_plant.slx
    save_system("ur5_plant");
    fprintf('Created Simscape model ur5_plant.slx (open it to see the 3D arm).\n');
end

%% 7. Save for later steps
save('arm_model.mat', 'p', 'robot', 'eeName', 'q_home', 'q_ready', 'hasRST', 'hasSM');
fprintf('Saved arm_model.mat - Step 1 complete.\n');

%% --- local function ---
function drawFrame(T, s)
    o = T(1:3,4);  R = T(1:3,1:3);
    c = 'rgb';
    for j = 1:3
        e = o + s*R(:,j);
        plot3([o(1) e(1)], [o(2) e(2)], [o(3) e(3)], c(j), 'LineWidth', 2, ...
              'HandleVisibility','off');
    end
end
