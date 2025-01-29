
function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 1 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 1);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 1);

%REFRENCE FOR ROCK GOAL
[ang_rock, lin_rock] = CartError(uvms.wTr , uvms.wTv);
uvms.xdot.rock_ang = 1 * ang_rock;
% display(uvms.xdot.rock_ang)
uvms.xdot.rock_ang = Saturate(uvms.xdot.rock_ang, 1);

%REFRENCE FOR VEHICLE POSITION CONTROL TASK
[ang_v, lin_v] = CartError(uvms.wTgv , uvms.wTv);
uvms.xdot.v_l = 1 * lin_v;
uvms.xdot.v_a = 1 * ang_v;
uvms.xdot.v_l = Saturate(uvms.xdot.v_l, 1);
uvms.xdot.v_a = Saturate(uvms.xdot.v_a, 1);

%TO MAKE IT HORIZONTAL
uvms.xdot.ha = 1 * (0 - norm(uvms.v_rho_ha)); % norm(v_rho) IS THETA

%TO MAINTAIN THE MINIMUM ALTITUDE
uvms.xdot.ma = 1* (1 - uvms.a);

%REFRENCE FOR THE ALTITUDE TASK
uvms.xdot.a = 1* (0 - uvms.a); %I WANT TO LAND ON THE SEA FLOOR

%REFRENCE FOR UNDERACTUATION TASK
uvms.xdot.und = uvms.p_dot;
 
uvms.xdot.stop = zeros(6,1);
