%% STEP3_COMPARE_NOMINAL  Step 3 - PID vs computed-torque control (perfect model)
%  Runs both controllers on the same pick-and-place trajectory with identical
%  closed-loop bandwidth, prints a comparison table and saves README figures.
%
%  Requires: build_ur5_model.m, simulate_ur5.m, arm_model.mat, trajectory.mat
%  Author: Harish Venkatesan M R

clear; clc; close all;
A = load('arm_model.mat', 'robot');
robot = A.robot;

resPID = simulate_ur5('PID', robot, robot, 'PID');
resCTC = simulate_ur5('CTC', robot, robot, 'Computed torque');

%% Comparison table
mP = resPID.metrics;  mC = resCTC.metrics;
fprintf('\n%-26s %12s %16s %12s\n', 'Metric', 'PID', 'Computed torque', 'Change');
row = @(name, a, b) fprintf('%-26s %12.3f %16.3f %11.1f%%\n', name, a, b, 100*(b-a)/a);
row('EE error RMS [mm]',   mP.rms_ee_mm,   mC.rms_ee_mm);
row('EE error max [mm]',   mP.max_ee_mm,   mC.max_ee_mm);
row('EE error final [mm]', mP.final_ee_mm, mC.final_ee_mm);
row('J2 error RMS [mrad]', mP.rms_joint_mrad(2), mC.rms_joint_mrad(2));
row('J3 error RMS [mrad]', mP.rms_joint_mrad(3), mC.rms_joint_mrad(3));
row('J2 peak torque [N*m]', mP.peak_tau(2), mC.peak_tau(2));
fprintf('Torque saturation hit:  PID %d | CTC %d\n', mP.saturated, mC.saturated);

%% Figures
c = lines(2);
f1 = figure('Name','EE error', 'Color','w', 'Position',[100 100 900 380]);
semilogy(resPID.tEE, 1e3*max(resPID.eeErr,1e-9), 'LineWidth',1.6, 'Color',c(1,:)); hold on;
semilogy(resCTC.tEE, 1e3*max(resCTC.eeErr,1e-9), 'LineWidth',1.6, 'Color',c(2,:));
grid on; xlabel('time [s]'); ylabel('end-effector error [mm]');
legend(sprintf('PID (RMS %.2f mm)', mP.rms_ee_mm), ...
       sprintf('Computed torque (RMS %.3f mm)', mC.rms_ee_mm), 'Location','best');
title('UR5 pick-and-place: end-effector tracking error');

f2 = figure('Name','Joint errors and torques', 'Color','w', 'Position',[100 100 900 600]);
for j = 2:3
    subplot(3,1,j-1);
    plot(resPID.t, 1e3*resPID.e(:,j), 'LineWidth',1.3, 'Color',c(1,:)); hold on;
    plot(resCTC.t, 1e3*resCTC.e(:,j), 'LineWidth',1.3, 'Color',c(2,:));
    grid on; ylabel(sprintf('J%d error [mrad]', j)); legend('PID','Computed torque');
end
subplot(3,1,3);
plot(resPID.t, resPID.tau(:,2), 'LineWidth',1.3, 'Color',c(1,:)); hold on;
plot(resCTC.t, resCTC.tau(:,2), '--', 'LineWidth',1.3, 'Color',c(2,:));
grid on; ylabel('J2 torque [N*m]'); xlabel('time [s]'); legend('PID','Computed torque');
sgtitle('Shoulder (J2) and elbow (J3): the gravity-loaded joints');

exportgraphics(f1, 'compare_nominal_ee.png', Resolution=150);
exportgraphics(f2, 'compare_nominal_joints.png', Resolution=150);
save('results_nominal.mat', 'resPID', 'resCTC');
fprintf('\nSaved results_nominal.mat and two PNGs - Step 3 complete.\n');
