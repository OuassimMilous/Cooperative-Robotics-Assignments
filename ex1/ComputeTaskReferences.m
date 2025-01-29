
function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% move
[ang_v, lin_v] = CartError(uvms.wTgv , uvms.wTv);
uvms.xdot.v = [ 1*lin_v;1*ang_v];
uvms.xdot.v = Saturate(uvms.xdot.v, 1);
% display(uvms.xdot.v)
% tool
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.tool = 1 * [ang; lin];
uvms.xdot.tool(1:3) = Saturate(uvms.xdot.tool(1:3), 1);
uvms.xdot.tool(4:6) = Saturate(uvms.xdot.tool(4:6), 1);

%rock
[ang_rock, lin_rock] = CartError(uvms.wTr , uvms.wTv);
uvms.xdot.rock = 1 * ang_rock;
uvms.xdot.rock = Saturate(uvms.xdot.rock, 1);


% horizental
uvms.xdot.ha = 1 * (0 - norm(uvms.v_rho_ha)); % norm(v_rho) IS THETA

% minimum altitude
uvms.xdot.ma = 1* (1 - uvms.a);

% landing
uvms.xdot.landing = 1* (0 - uvms.a);

% underactuation
uvms.xdot.und = uvms.p_dot;
 
% stop
uvms.xdot.stop = zeros(6,1);
