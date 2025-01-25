function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

%REFRENCE FOR ROCK GOAL
[ang_rock, lin_rock] = CartError(uvms.wTr , uvms.wTv);
uvms.xdot.rock_ang = 0.2 * ang_rock;
% display(uvms.xdot.rock_ang)
uvms.xdot.rock_ang = Saturate(uvms.xdot.rock_ang, 0.2);

%REFRENCE FOR VEHICLE POSITION CONTROL TASK
[ang_v, lin_v] = CartError(uvms.wTgv , uvms.wTv);
uvms.xdot.v_l = 0.2 * lin_v;
uvms.xdot.v_a = 0.2 * ang_v;
uvms.xdot.v_l = Saturate(uvms.xdot.v_l, 0.2);
uvms.xdot.v_a = Saturate(uvms.xdot.v_a, 0.2);

%TO MAKE IT HORIZONTAL
uvms.xdot.ha = 0.2 * (0 - norm(uvms.v_rho_ha)); % norm(v_rho) IS THETA

%TO MAINTAIN THE MINIMUM ALTITUDE
uvms.xdot.ma = 0.2* (1 - uvms.a);

%REFRENCE FOR THE ALTITUDE TASK
uvms.xdot.a = 0.2* (0 - uvms.a); %I WANT TO LAND ON THE SEA FLOOR

%REFRENCE FOR UNDERACTUATION TASK
uvms.xdot.und = uvms.p_dot;
 
