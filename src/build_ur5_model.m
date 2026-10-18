function mdl = build_ur5_model(controller)
%BUILD_UR5_MODEL  Build the closed-loop UR5 Simulink model for a given controller.
%   mdl = build_ur5_model('PID')  -> ur5_pid.slx
%   mdl = build_ur5_model('CTC')  -> ur5_ctc.slx  (computed-torque control)
%
%   PID :  tau = KpP.*e + KdP.*ed + KiP.*int(e)
%   CTC :  a   = qddRef + KpC.*e + KdC.*ed + KiC.*int(e)
%          tau = M(q)*a + C(q,qd)*qd + G(q)        (Inverse Dynamics block, model 'robotCtrl')
%
%   Plant (both): Forward Dynamics block using 'robotPlant', torque saturation at tauMax.
%   Sensors: the controller sees q + sigmaQ*noise and qd + sigmaQd*noise (zero-mean,
%   unit-variance, 1 kHz). The logged q/qd are the TRUE states.
%
%   Workspace variables the model expects (set by simulate_ur5.m):
%   qRef qdRef qddRef Tend q0 tauMax NB NBc robotPlant robotCtrl
%   KpP KdP KiP KpC KdC KiC sigmaQ sigmaQd
%
%   Author: Harish Venkatesan M R

controller = upper(string(controller));
assert(any(controller == ["PID","CTC"]), 'controller must be ''PID'' or ''CTC''.');
mdl = char("ur5_" + lower(controller));

if bdIsLoaded(mdl), close_system(mdl, 0); end
if isfile([mdl '.slx']), delete([mdl '.slx']); end
new_system(mdl);
load_system('robotmaniplib');

B  = @(n) [mdl '/' n];
EW = 'Element-wise(K.*u)';
FW = 'simulink/Sources/From Workspace';
hold_ = {'OutputAfterFinalValue','Holding final value'};
RN = 'simulink/Sources/Random Number';

%% Reference + error blocks
add_block(FW, B('qRef'),  'VariableName','qRef',  hold_{:}, 'Position',[30  40 110  70]);
add_block(FW, B('qdRef'), 'VariableName','qdRef', hold_{:}, 'Position',[30 140 110 170]);
add_block('simulink/Math Operations/Sum', B('e'),  'Inputs','+-', 'Position',[170  45 190  65]);
add_block('simulink/Math Operations/Sum', B('ed'), 'Inputs','+-', 'Position',[170 145 190 165]);
add_block('simulink/Continuous/Integrator', B('int e'), 'InitialCondition','zeros(6,1)', ...
          'Position',[240 90 270 120]);

%% Controller
if controller == "PID"
    add_block('simulink/Math Operations/Gain', B('Kp'), 'Gain','KpP', 'Multiplication',EW, 'Position',[310  40 360  70]);
    add_block('simulink/Math Operations/Gain', B('Ki'), 'Gain','KiP', 'Multiplication',EW, 'Position',[310  90 360 120]);
    add_block('simulink/Math Operations/Gain', B('Kd'), 'Gain','KdP', 'Multiplication',EW, 'Position',[310 140 360 170]);
    add_block('simulink/Math Operations/Sum', B('tau PID'), 'Inputs','+++', 'Position',[410 75 430 135]);
    ctrlOut = 'tau PID/1';
else
    add_block(FW, B('qddRef'), 'VariableName','qddRef', hold_{:}, 'Position',[30 230 110 260]);
    add_block('simulink/Math Operations/Gain', B('Kp'), 'Gain','KpC', 'Multiplication',EW, 'Position',[310  40 360  70]);
    add_block('simulink/Math Operations/Gain', B('Ki'), 'Gain','KiC', 'Multiplication',EW, 'Position',[310  90 360 120]);
    add_block('simulink/Math Operations/Gain', B('Kd'), 'Gain','KdC', 'Multiplication',EW, 'Position',[310 140 360 170]);
    add_block('simulink/Math Operations/Sum', B('a cmd'), 'Inputs','++++', 'Position',[410 70 430 190]);
    add_block('robotmaniplib/Inverse Dynamics', B('Model Inverse Dynamics'), 'Position',[470 70 600 190]);
    setTreeParam(B('Model Inverse Dynamics'), 'robotCtrl');
    useCodegen(B('Model Inverse Dynamics'));
    add_block('simulink/Sources/Constant', B('Fext ctrl'), 'Value','zeros(6,NBc)', 'Position',[400 230 460 260]);
    ctrlOut = 'Model Inverse Dynamics/1';
end

%% Plant
add_block('simulink/Discontinuities/Saturation', B('Torque limits'), ...
          'UpperLimit','tauMax', 'LowerLimit','-tauMax', 'Position',[650 115 690 145]);
add_block('robotmaniplib/Forward Dynamics', B('Plant Forward Dynamics'), 'Position',[740 80 880 180]);
setTreeParam(B('Plant Forward Dynamics'), 'robotPlant');
useCodegen(B('Plant Forward Dynamics'));
add_block('simulink/Sources/Constant', B('Fext plant'), 'Value','zeros(6,NB)', 'Position',[650 200 710 230]);
add_block('simulink/Continuous/Integrator', B('qd'), 'InitialCondition','zeros(6,1)', 'Position',[930 115 960 145]);
add_block('simulink/Continuous/Integrator', B('q'),  'InitialCondition','q0',         'Position',[1010 115 1040 145]);

%% Sensors (measurement noise on what the controller sees)
add_block(RN, B('noise q'),  'Mean','zeros(6,1)', 'Variance','ones(6,1)', ...
          'Seed','[11 12 13 14 15 16]', 'SampleTime','1e-3', 'Position',[1010 300 1050 330]);
add_block('simulink/Math Operations/Gain', B('sigma q'), 'Gain','sigmaQ', 'Position',[1080 300 1120 330]);
add_block('simulink/Math Operations/Sum', B('q meas'), 'Inputs','++', 'Position',[1150 260 1170 280]);
add_block(RN, B('noise qd'), 'Mean','zeros(6,1)', 'Variance','ones(6,1)', ...
          'Seed','[21 22 23 24 25 26]', 'SampleTime','1e-3', 'Position',[1010 380 1050 410]);
add_block('simulink/Math Operations/Gain', B('sigma qd'), 'Gain','sigmaQd', 'Position',[1080 380 1120 410]);
add_block('simulink/Math Operations/Sum', B('qd meas'), 'Inputs','++', 'Position',[1150 340 1170 360]);

%% Logging (true states)
add_block('simulink/Sinks/To Workspace', B('log q'),   'VariableName','q_log',   'SaveFormat','Timeseries', 'Position',[1100 115 1160 145]);
add_block('simulink/Sinks/To Workspace', B('log qd'),  'VariableName','qd_log',  'SaveFormat','Timeseries', 'Position',[1100 180 1160 210]);
add_block('simulink/Sinks/To Workspace', B('log tau'), 'VariableName','tau_log', 'SaveFormat','Timeseries', 'Position',[740 230 800 260]);
add_block('simulink/Sinks/To Workspace', B('log q meas'), 'VariableName','qmeas_log', 'SaveFormat','Timeseries', 'Position',[1220 255 1280 285]);

%% Wiring
L = @(a,b) add_line(mdl, a, b, 'autorouting','smart');
% sensors
L('q/1','q meas/1');    L('noise q/1','sigma q/1');   L('sigma q/1','q meas/2');
L('qd/1','qd meas/1');  L('noise qd/1','sigma qd/1'); L('sigma qd/1','qd meas/2');
% errors use MEASURED states
L('qRef/1','e/1');    L('q meas/1','e/2');
L('qdRef/1','ed/1');  L('qd meas/1','ed/2');
L('e/1','int e/1');
L('e/1','Kp/1');      L('int e/1','Ki/1');     L('ed/1','Kd/1');

if controller == "PID"
    L('Kp/1','tau PID/1');  L('Ki/1','tau PID/2');  L('Kd/1','tau PID/3');
else
    L('qddRef/1','a cmd/1'); L('Kp/1','a cmd/2'); L('Kd/1','a cmd/3'); L('Ki/1','a cmd/4');
    id = 'Model Inverse Dynamics';
    connectPort(mdl, 'q meas/1',    id, 'config', 1, true);
    connectPort(mdl, 'qd meas/1',   id, 'vel',    2, true);
    connectPort(mdl, 'a cmd/1',     id, 'accel',  3, true);
    connectPort(mdl, 'Fext ctrl/1', id, 'ext',    4, false);
end

L(ctrlOut, 'Torque limits/1');
fd = 'Plant Forward Dynamics';               % plant uses TRUE states
connectPort(mdl, 'q/1',             fd, 'config', 1, true);
connectPort(mdl, 'qd/1',            fd, 'vel',    2, true);
connectPort(mdl, 'Torque limits/1', fd, 'torq',   3, true);
connectPort(mdl, 'Fext plant/1',    fd, 'ext',    4, false);
L([fd '/1'], 'qd/1');
L('qd/1', 'q/1');

L('q/1','log q/1');  L('qd/1','log qd/1');  L('Torque limits/1','log tau/1');
L('q meas/1','log q meas/1');

set_param(mdl, 'Solver','ode45', 'StopTime','Tend', 'MaxStep','1e-3', ...
          'RelTol','1e-5', 'ReturnWorkspaceOutputs','on');
save_system(mdl);
end

%% ---------- local helpers ----------
function setTreeParam(blk, value)
    dp  = fieldnames(get_param(blk, 'DialogParameters'));
    hit = dp(contains(lower(dp), 'tree'));
    assert(~isempty(hit), 'No rigidBodyTree parameter found on %s', blk);
    set_param(blk, hit{1}, value);
end

function useCodegen(blk)
    try
        dp  = fieldnames(get_param(blk, 'DialogParameters'));
        hit = dp(contains(lower(dp), 'simulat'));
        if ~isempty(hit), set_param(blk, hit{1}, 'Code generation'); end
    catch
    end
end

function connectPort(mdl, src, blkName, key, default, required)
    n = portNum([mdl '/' blkName], key, default);
    if isnan(n)
        assert(~required, 'Could not find input "%s" on %s', key, blkName);
        return
    end
    add_line(mdl, src, sprintf('%s/%d', blkName, n), 'autorouting','smart');
end

function n = portNum(blk, key, default)
    n = NaN;
    try
        h = find_system(blk, 'LookUnderMasks','all', 'FollowLinks','on', ...
                        'SearchDepth',1, 'BlockType','Inport');
        if ~isempty(h)
            names = lower(string(get_param(h, 'Name')));
            i = find(contains(names, key), 1);
            if ~isempty(i), n = str2double(get_param(h{i}, 'Port')); end
            return
        end
    catch
    end
    pc = get_param(blk, 'Ports');
    if default <= pc(1), n = default; end
end
