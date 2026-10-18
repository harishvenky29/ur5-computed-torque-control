%% RUN_ALL  Reproduce the whole UR5 PID vs computed-torque project
%  step1_model_arm.m         -> arm_model.mat
%  step2_plan_trajectory.m   -> trajectory.mat, arm_pick_place.gif
%  step3_compare_nominal.m   -> results_nominal.mat, compare_nominal_*.png
%  step4_robustness_study.m  -> results_robustness.*, results_table.md, robustness_*.png
%
%  Requirements: see requirements.txt (checked automatically below). About 6-8 minutes.
%  Each step script starts with "clear", so the steps are called explicitly
%  rather than in a loop (a loop variable would be wiped).
%  Author: Harish Venkatesan M R

if ~check_requirements()
    error('run_all:requirements', 'Missing required MATLAB products - see requirements.txt.');
end

run('step1_model_arm.m');
run('step2_plan_trajectory.m');
run('step3_compare_nominal.m');
run('step4_robustness_study.m');

fprintf('\nAll steps complete. Results table: results_table.md\n');
