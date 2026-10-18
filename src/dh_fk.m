function [T, Tall] = dh_fk(q, p)
%DH_FK  Forward kinematics of a serial arm from a standard DH table.
%   [T, Tall] = dh_fk(q, p)
%   q    : 6x1 or 1x6 joint angles [rad]
%   p    : parameter struct from ur5_params()
%   T    : 4x4 base-to-end-effector homogeneous transform
%   Tall : 4x4x(n+1) transforms of every frame (frame 0 = base), for plotting
%
%   Pure MATLAB - no toolboxes required. Compatible with MATLAB Function blocks.

n = p.n;
Tall = zeros(4,4,n+1);
Tall(:,:,1) = eye(4);
T = eye(4);
for i = 1:n
    th = q(i) + p.offset(i);
    ct = cos(th);  st = sin(th);
    ca = cos(p.alpha(i));  sa = sin(p.alpha(i));
    A = [ct, -st*ca,  st*sa, p.a(i)*ct;
         st,  ct*ca, -ct*sa, p.a(i)*st;
          0,     sa,     ca,     p.d(i);
          0,      0,      0,          1];
    T = T * A;
    Tall(:,:,i+1) = T;
end
end
