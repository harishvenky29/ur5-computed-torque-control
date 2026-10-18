function res = simulate_ur5(controller, robotPlant, robotCtrl, label, opts)
%SIMULATE_UR5  Build, run and score one closed-loop UR5 simulation.
%   res = simulate_ur5(controller, robotPlant, robotCtrl, label, opts)
%   controller : 'PID' or 'CTC'
%   robotPlant : rigidBodyTree used as the simulated (real) arm
%   robotCtrl  : rigidBodyTree the controller believes in (gains / inverse dynamics)
%   label      : name for printouts (optional)
%   opts       : struct, optional fields
%                  sigmaQ  - joint angle sensor noise std    [rad]   (default 0)
%                  sigmaQd - joint speed sensor noise std    [rad/s] (default 0)
%
%   Both controllers get the same nominal closed-loop design
%   (s^3 + 2*zeta*wn s^2 + wn^2 s + wn^3/10, wn = 20 rad/s), so the comparison is fair.
%
%   Requires trajectory.mat (step2) and arm_model.mat (step1).
%   Author: Harish Venkatesan M R

if nargin < 4 || isempty(label), label = controller; end
if nargin < 5, opts = struct; end
if ~isfield(opts, 'sigmaQ'),  opts.sigmaQ  = 0; end
if ~isfield(opts, 'sigmaQd'), opts.sigmaQd = 0; end

S = load('trajectory.mat', 'qRef','qdRef','qddRef','qWp','tSamples','q','Tend');
P = load('arm_model.mat', 'p', 'eeName');

%% Gains (designed on the controller's model, never the plant)
q0 = S.qWp(:,1);
wn = 20; zeta = 1;
Mdiag = diag(massMatrix(robotCtrl, q0));
v.KpP = Mdiag * wn^2;
v.KdP = Mdiag * 2*zeta*wn;
v.KiP = v.KpP * wn/10;
v.KpC = wn^2        * ones(6,1);
v.KdC = 2*zeta*wn   * ones(6,1);
v.KiC = (wn^3/10)   * ones(6,1);

%% Everything the Simulink model reads from the workspace
v.qRef = S.qRef;  v.qdRef = S.qdRef;  v.qddRef = S.qddRef;
v.Tend = S.Tend;  v.q0 = q0;          v.tauMax = P.p.tau_max(:);
v.NB = robotPlant.NumBodies;          v.NBc = robotCtrl.NumBodies;
v.robotPlant = robotPlant;            v.robotCtrl = robotCtrl;
v.sigmaQ = opts.sigmaQ;               v.sigmaQd = opts.sigmaQd;
f = fieldnames(v);
for i = 1:numel(f), assignin('base', f{i}, v.(f{i})); end

%% Build + simulate
mdl = build_ur5_model(controller);
assert(~isempty(find_system(mdl, 'SearchDepth',1, 'Name','q meas')), ...
    ['Model has no sensor-noise blocks, so an OLD build_ur5_model.m is being used:\n  %s\n' ...
     'Replace that file with the latest version (and delete any other copies on the path).'], ...
    which('build_ur5_model'));
fprintf('[%s] simulating %.1f s ... ', label, S.Tend);
tic; out = sim(mdl); wall = toc;
fprintf('done (%.1f s wall)\n', wall);

%% Score (true states vs reference)
t    = out.q_log.Time;
qSim = tsData(out.q_log);
tau  = tsData(out.tau_log);
qDes = interp1(S.tSamples', S.q', t);
e    = qDes - qSim;

% Verify the noise the controller actually received
tm   = out.qmeas_log.Time;
qMeas = tsData(out.qmeas_log);
noiseStd = std(qMeas - interp1(t, qSim, tm), 0, 'all');
fprintf('[%s] sensor noise seen by controller: std %.3g rad (requested %.3g)\n', ...
        label, noiseStd, opts.sigmaQ);
if opts.sigmaQ > 0 && noiseStd < 0.1*opts.sigmaQ
    warning('Requested sensor noise is NOT reaching the controller.');
end

k = round(linspace(1, numel(t), 400));
eeErr = zeros(numel(k), 1);
for i = 1:numel(k)
    Td = getTransform(robotCtrl, qDes(k(i),:)', P.eeName);
    Ta = getTransform(robotCtrl, qSim(k(i),:)', P.eeName);
    eeErr(i) = norm(Td(1:3,4) - Ta(1:3,4));
end

m.rms_joint_mrad = 1e3 * sqrt(mean(e.^2, 1));
m.max_joint_mrad = 1e3 * max(abs(e), [], 1);
m.rms_ee_mm   = 1e3 * sqrt(mean(eeErr.^2));
m.max_ee_mm   = 1e3 * max(eeErr);
m.final_ee_mm = 1e3 * eeErr(end);
m.peak_tau    = max(abs(tau), [], 1);
m.tau_jitter  = mean(abs(diff(tau)), 'all');      % torque chatter indicator
m.saturated   = any(abs(tau) >= v.tauMax' - 1e-6, 'all');

res = struct('label',label, 'controller',controller, 't',t, 'qSim',qSim, 'qDes',qDes, ...
             'e',e, 'tau',tau, 'tEE',t(k), 'eeErr',eeErr, 'metrics',m, 'wall',wall);

fprintf('[%s] EE error RMS %.4g mm | max %.4g mm | final %.4g mm | peak tau %s\n', ...
        label, m.rms_ee_mm, m.max_ee_mm, m.final_ee_mm, mat2str(m.peak_tau, 3));
end

function d = tsData(ts)
    d = ts.Data;
    if ndims(d) == 3, d = squeeze(d)'; end
    if size(d,2) ~= 6 && size(d,1) == 6, d = d'; end
end
