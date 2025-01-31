function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% move
[ang_v, lin_v] = CartError(uvms.wTgv , uvms.wTv);
uvms.xdot.v = [ 1*lin_v;1*ang_v];
uvms.xdot.v = Saturate(uvms.xdot.v, 1);
% display(uvms.xdot.v)
% tool
[ang, lin] = CartError(uvms.wTg , uvms.wTt);
uvms.xdot.tool = 1 * [ang; lin];
uvms.xdot.tool(1:3) = Saturate(uvms.xdot.tool(1:3), 1);
uvms.xdot.tool(4:6) = Saturate(uvms.xdot.tool(4:6), 1);

% closer

[ang_closer, uvms.err.lin_closer] = CartError(uvms.wTg , uvms.wTt);
uvms.xdot.closer = [uvms.err.lin_closer(1:2);0];
uvms.xdot.closer = Saturate(uvms.xdot.closer, 1);

%rock
% [ang_rock, lin_rock] = CartError(uvms.wTg , uvms.wTv);


% Compute target vector in the XY plane
target_vector = uvms.goalPosition(1:2) - uvms.p(1:2);

% Normalize the target vector
if norm(target_vector) == 0
    error('Target vector is zero, cannot compute yaw angle.');
end
unit_vector = target_vector / norm(target_vector);

% Compute desired yaw angle
yaw_target = atan2(unit_vector(2), unit_vector(1));

% Compute current yaw angle from the rotation matrix
yaw_current = atan2(uvms.wTv(2, 1), uvms.wTv(1, 1));

% Compute yaw error
uvms.err.rock =  yaw_current - yaw_target;

% Normalize yaw error to [-pi, pi]
uvms.err.rock  = mod(uvms.err.rock + pi, 2*pi) - pi;

% Compute velocity control for yaw
uvms.xdot.rock = [0;0;1 * uvms.err.rock]; 

uvms.xdot.rock = Saturate(uvms.xdot.rock, -1);

% horizental
uvms.xdot.ha = 1 * (0 - norm(uvms.v_rho_ha)); % norm(v_rho) IS THETA

% minimum altitude
uvms.xdot.ma = 1* (1 - uvms.a);

% landing
uvms.xdot.landing = 0.8* (0 - uvms.a);

% underactuation
uvms.xdot.und = uvms.p_dot;
 
% stop
uvms.xdot.stop = zeros(6,1);
