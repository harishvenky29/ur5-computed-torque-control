%% STEP4_ROBUSTNESS_STUDY  Step 4 - PID vs computed torque when the model is wrong
%  The controller always uses the NOMINAL UR5 model. The simulated arm gets:
%    1. Nominal                      - plant = model (sanity check)
%    2. 2 kg payload                 - unknown mass at the tool (gripping a part)
%    3. Payload + 10% link mass error - CAD/URDF masses are never exact
%    4. ... + sensor noise           - encoder noise on q and noisy velocity estimate
%  Prints a results table, writes a CSV + a Markdown table for the README,
%  and saves comparison figures.
%
%  Requires: build_ur5_model.m, simulate_ur5.m, arm_model.mat, trajectory.mat
%  Runtime: 8 simulations, roughly 5-6 minutes.
%  Author: Harish Venkatesan M R

clear; clc; close all;
A = load('arm_model.mat', 'robot', 'eeName');
robot = A.robot;  eeName = A.eeName;

%% Scenarios
sc = struct( ...
  'name',      {'Nominal', '2 kg payload', 'Payload + 10% mass error', 'Payload + mass error + noise'}, ...
  'payload',   {0,         2,              2,                          2}, ...
  'massScale', {1,         1,              1.10,                       1.10}, ...
  'sigmaQ',    {0,         0,              0,                          1e-4}, ...   % ~0.006 deg encoder noise
  'sigmaQd',   {0,         0,              0,                          2e-3});      % rad/s velocity estimate noise
nS = numel(sc);
ctrls = {'PID', 'CTC'};
R = cell(nS, 2);

for s = 1:nS
    fprintf('\n=== Scenario %d/%d: %s ===\n', s, nS, sc(s).name);
    plant = makePlant(robot, eeName, sc(s).payload, sc(s).massScale);
    opts = struct('sigmaQ', sc(s).sigmaQ, 'sigmaQd', sc(s).sigmaQd);
    for c = 1:2
        R{s,c} = simulate_ur5(ctrls{c}, plant, robot, sprintf('%s | %s', ctrls{c}, sc(s).name), opts);
    end
end

%% Results table
names = string({sc.name})';
get = @(c, f) cellfun(@(r) r.metrics.(f), R(:,c));
pidRMS = get(1,'rms_ee_mm');   ctcRMS = get(2,'rms_ee_mm');
pidMax = get(1,'max_ee_mm');   ctcMax = get(2,'max_ee_mm');
pidFin = get(1,'final_ee_mm'); ctcFin = get(2,'final_ee_mm');
pidJit = get(1,'tau_jitter');  ctcJit = get(2,'tau_jitter');
redRMS = 100 * (1 - ctcRMS ./ pidRMS);
redMax = 100 * (1 - ctcMax ./ pidMax);

T = table(names, pidRMS, ctcRMS, redRMS, pidMax, ctcMax, redMax, pidFin, ctcFin, pidJit, ctcJit, ...
    'VariableNames', {'Scenario','PID_RMS_mm','CTC_RMS_mm','RMS_reduction_pct', ...
                      'PID_max_mm','CTC_max_mm','Max_reduction_pct', ...
                      'PID_final_mm','CTC_final_mm','PID_tau_jitter','CTC_tau_jitter'});
disp(T);
writetable(T, 'results_robustness.csv');

% Markdown table to paste into the README
fid = fopen('results_table.md', 'w');
fprintf(fid, '| Scenario | PID RMS [mm] | CTC RMS [mm] | RMS reduction | PID max [mm] | CTC max [mm] |\n');
fprintf(fid, '|---|---|---|---|---|---|\n');
for s = 1:nS
    fprintf(fid, '| %s | %.3g | %.3g | %.1f%% | %.3g | %.3g |\n', ...
        names(s), pidRMS(s), ctcRMS(s), redRMS(s), pidMax(s), ctcMax(s));
end
fclose(fid);
fprintf('\n%s\n', fileread('results_table.md'));

%% Figures
c = lines(2);
f1 = figure('Name','RMS error by scenario', 'Color','w', 'Position',[100 100 900 420]);
b = bar(categorical(names, names), max([pidRMS ctcRMS], 1e-6));
b(1).FaceColor = c(1,:);  b(2).FaceColor = c(2,:);
set(gca, 'YScale','log'); grid on;
ylabel('end-effector RMS error [mm]'); legend('PID', 'Computed torque', 'Location','northwest');
title('Tracking error vs model mismatch (controller always uses the nominal model)');

worst = nS;
f2 = figure('Name','Worst case', 'Color','w', 'Position',[100 100 900 380]);
plot(R{worst,1}.tEE, 1e3*R{worst,1}.eeErr, 'LineWidth',1.5, 'Color',c(1,:)); hold on;
plot(R{worst,2}.tEE, 1e3*R{worst,2}.eeErr, 'LineWidth',1.5, 'Color',c(2,:));
grid on; xlabel('time [s]'); ylabel('end-effector error [mm]');
legend(sprintf('PID (RMS %.2f mm)', pidRMS(worst)), ...
       sprintf('Computed torque (RMS %.2f mm)', ctcRMS(worst)), 'Location','best');
title(sprintf('Worst case: %s', names(worst)));

exportgraphics(f1, 'robustness_rms.png', Resolution=150);
exportgraphics(f2, 'robustness_worst_case.png', Resolution=150);
save('results_robustness.mat', 'R', 'sc', 'T');
fprintf('Saved results_robustness.mat/.csv, results_table.md and two PNGs - Step 4 complete.\n');

%% --- local function ---
function plant = makePlant(robot, eeName, payloadKg, massScale)
    % Copy of the nominal arm with scaled link masses and an optional payload
    plant = copy(robot);
    if massScale ~= 1
        for i = 1:plant.NumBodies
            name = plant.BodyNames{i};
            b = copy(getBody(plant, name));
            if b.Mass > 0
                b.Mass    = b.Mass    * massScale;
                b.Inertia = b.Inertia * massScale;
                replaceBody(plant, name, b);
            end
        end
    end
    if payloadKg > 0
        r = 0.05;                                   % payload modelled as a 5 cm sphere
        com = [0 0 0.05];                           % 5 cm out from the tool flange
        Ic  = 2/5 * payloadKg * r^2;                % sphere inertia about its CoM
        % rigidBody.Inertia is about the body frame origin -> parallel-axis theorem
        Ixx = Ic + payloadKg*(com(2)^2 + com(3)^2);
        Iyy = Ic + payloadKg*(com(1)^2 + com(3)^2);
        Izz = Ic + payloadKg*(com(1)^2 + com(2)^2);
        body = rigidBody('payload');
        body.Mass = payloadKg;
        body.CenterOfMass = com;
        body.Inertia = [Ixx Iyy Izz 0 0 0];
        body.Joint = rigidBodyJoint('payload_joint', 'fixed');
        addBody(plant, body, eeName);
    end
    totalNom = sum(cellfun(@(b) b.Mass, robot.Bodies));
    totalPl  = sum(cellfun(@(b) b.Mass, plant.Bodies));
    fprintf('Plant mass %.2f kg (model believes %.2f kg)\n', totalPl, totalNom);
end
