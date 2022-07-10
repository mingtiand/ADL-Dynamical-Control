clear all;clc;close all;
Xt_x = 40;
Xt_y = 20;
Xt_z = 0;

epsilon = 2; 
alpha = 10;
lamda = 0.002;
q_initial = [0 -pi/2 0 0 0 0 0];

O = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
r_shoulder2target_xy =  [Xt_x;Xt_y] - [0;0];
r_shoulder2target_yz = [Xt_y;Xt_z] - [0;0]; %Ey12(1);Ez12(1)
theta = acos(sum(r_shoulder2target_xy .*[0;1])/(norm(r_shoulder2target_xy)*norm([0;1])))
gamma = acos(sum(r_shoulder2target_yz .*[1;0])/(norm(r_shoulder2target_yz)*norm([1;0])))

%stop
if Xt_x>0
    theta = -theta;
end
if Xt_z<0
    gamma = -gamma;
end
O = [1 0 0 0;0 cos(gamma) -sin(gamma) 0;0 sin(gamma) cos(gamma) 0;0 0 0 1]*O;
O = [cos(theta) -sin(theta) 0 0;sin(theta) cos(theta) 0 0;0 0 1 0;0 0 0 1]*O;

fix = 1;
[Ex11 Ey11 Ez11 Wx11 Wy11 Wz11 X11 Y11 Z11 q1m11 q2m11 q3m11 q4m11 q5m11 q6m11 q7m11] = rg(O,Xt_x,Xt_y,Xt_z,alpha,lamda,q_initial,epsilon,fix);
[Ex11 Ey11 Ez11 Wx11 Wy11 Wz11 t_sample t_joint] = minimumjerk2func([Wx11;Wy11;Wz11],[Ex11;Ey11;Ez11]);