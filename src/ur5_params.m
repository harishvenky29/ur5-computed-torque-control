function p = ur5_params()
%UR5_PARAMS  Kinematic (standard DH) and inertial parameters of the UR5 arm.
%   Source: Universal Robots published DH parameters for the UR5.
%   Used by dh_fk.m (no toolbox needed) and later for the dynamics/control steps.

% Standard DH table: [a  alpha  d  theta_offset]
p.a     = [0, -0.425, -0.39225, 0, 0, 0];          % link lengths   [m]
p.alpha = [pi/2, 0, 0, pi/2, -pi/2, 0];            % link twists    [rad]
p.d     = [0.089159, 0, 0, 0.10915, 0.09465, 0.0823]; % link offsets [m]
p.offset = zeros(1,6);                             % joint zero offsets [rad]

% Link masses [kg]
p.m = [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879];

% Joint limits [rad] and max joint torques [N*m] (for saturation later)
p.qmin = -2*pi*ones(1,6);
p.qmax =  2*pi*ones(1,6);
p.tau_max = [150, 150, 150, 28, 28, 28];

p.n = 6;
end
