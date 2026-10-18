function ok = check_requirements()
%CHECK_REQUIREMENTS  Verify the MATLAB release and toolboxes this project needs.
%   ok = check_requirements()  prints a report and returns true if everything
%   required is installed and licensed. See requirements.txt.
%
%   Author: Harish Venkatesan M R

fprintf('\n=== Checking requirements ===\n');
ok = true;

% MATLAB release (R2022a = version 9.12)
rel = version('-release');
if verLessThan('matlab', '9.12')
    fprintf('  [FAIL] MATLAB R%s (need R2022a or newer)\n', rel);
    ok = false;
else
    fprintf('  [ OK ] MATLAB R%s\n', rel);
end

% Required products: {display name, ver() name, license feature}
req = { 'Simulink',                'Simulink',                'SIMULINK';
        'Robotics System Toolbox', 'Robotics System Toolbox', 'Robotics_System_Toolbox' };
opt = { 'Simscape Multibody',      'Simscape Multibody',      'Simscape_Multibody' };

for i = 1:size(req,1)
    [inst, lic] = hasProduct(req{i,2}, req{i,3});
    if inst && lic
        fprintf('  [ OK ] %s\n', req{i,1});
    else
        fprintf('  [FAIL] %s (installed: %d, licensed: %d)\n', req{i,1}, inst, lic);
        ok = false;
    end
end
for i = 1:size(opt,1)
    [inst, lic] = hasProduct(opt{i,2}, opt{i,3});
    if inst && lic
        fprintf('  [ OK ] %s (optional)\n', opt{i,1});
    else
        fprintf('  [ -- ] %s not available (optional, only for the 3D model)\n', opt{i,1});
    end
end

if ok
    fprintf('All required products found.\n\n');
else
    fprintf('Missing requirements - install them via Home > Add-Ons, then re-run.\n\n');
end
end

function [inst, lic] = hasProduct(verName, licName)
    v = ver;
    inst = any(strcmp({v.Name}, verName));
    lic  = license('test', licName) == 1;
end
