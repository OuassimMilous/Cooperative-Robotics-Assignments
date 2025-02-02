function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here


% horizental
uvms.xdot.ha = 1 * (0 - norm(uvms.v_rho_ha)); % norm(v_rho) IS THETA

% minimum altitude
uvms.xdot.ma = 1* (2 - uvms.a);

if mission.phase == 1
    % move
    [uvms.err.ang_v, uvms.err.lin_v] = CartError(uvms.wTgv , uvms.wTv);
    uvms.xdot.v = [ 1*uvms.err.lin_v;1*uvms.err.ang_v];
    uvms.xdot.v = Saturate(uvms.xdot.v, 1);

elseif mission.phase ==2

    target_vector = uvms.goalPosition(1:2) - uvms.p(1:2); % Compute target vector in the XY plane

    % Normalize the target vector
    if norm(target_vector) == 0
        error('Target vector is zero, cannot compute yaw angle.');
    end
    unit_vector = target_vector / norm(target_vector);
    yaw_target = atan2(unit_vector(2), unit_vector(1)); % Compute desired yaw angle
    yaw_current = atan2(uvms.wTv(2, 1), uvms.wTv(1, 1)); % Compute current yaw angle from the rotation matrix
    uvms.err.rock =  yaw_target-yaw_current; % Compute yaw error
    uvms.err.rock  = mod(uvms.err.rock + pi, 2*pi) - pi; % Normalize yaw error to [-pi, pi]

    uvms.xdot.v = [0;0;0; 0;0; 1 * uvms.err.rock];  % Compute velocity control for yaw gain is 1/uvms.wTv(3,3) because of the jacobian

    uvms.xdot.v = Saturate(uvms.xdot.v, 1);

elseif mission.phase == 4
    uvms.xdot.v = zeros(6,1);
    uvms.xdot.v = Saturate(uvms.xdot.v, 1);
else
end


% get closer
[ang_v,uvms.err.lin_closer] = CartError(uvms.wTg , uvms.wTt);
uvms.err.lin_closer = uvms.err.lin_closer(1:2);
uvms.xdot.closer = [[uvms.err.lin_closer; 0 ]];
uvms.xdot.closer = Saturate(uvms.xdot.closer, 1);

% landing
uvms.xdot.landing = 0.8* (0 - uvms.a);

% tool
[uvms.err.ang_t,  uvms.err.lin_t] = CartError(uvms.vTg , uvms.vTt);

uvms.xdot.tool = 1 * [ uvms.err.ang_t;  uvms.err.lin_t];
uvms.xdot.tool(1:3) = Saturate(uvms.xdot.tool(1:3), 1);
uvms.xdot.tool(4:6) = Saturate(uvms.xdot.tool(4:6), 1);


