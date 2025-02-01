function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here



if mission.phase == 1
    % move
    [uvms.err.ang_v, uvms.err.lin_v] = CartError(uvms.wTgv , uvms.wTv);
    uvms.xdot.v = [ 1*uvms.err.lin_v;1*uvms.err.ang_v];
    uvms.xdot.v = Saturate(uvms.xdot.v, 1);
elseif mission.phase == 3
    [ang_v,uvms.err.lin_closer] = CartError(uvms.wTg , uvms.wTt);
    uvms.err.lin_closer = uvms.err.lin_closer(1:2);
    uvms.xdot.v = [[uvms.err.lin_closer; 0 ];zeros(3,1)];
    uvms.xdot.v = Saturate(uvms.xdot.v, 1);
elseif mission.phase == 4
    uvms.xdot.v = zeros(6,1);
    uvms.xdot.v = Saturate(uvms.xdot.v, 1);
else
end

% display(uvms.xdot.v)
% tool
[uvms.err.ang_t,  uvms.err.lin_t] = CartError(uvms.vTg , uvms.vTt);

uvms.xdot.tool = 1 * [ uvms.err.ang_t;  uvms.err.lin_t];
uvms.xdot.tool(1:3) = Saturate(uvms.xdot.tool(1:3), 1);
uvms.xdot.tool(4:6) = Saturate(uvms.xdot.tool(4:6), 1);


%rock
% [ang_rock, lin_rock] = CartError(uvms.wTg , uvms.wTv);

uvms.err.rock = norm(uvms.r_rho_ra);
uvms.xdot.rock = [1 * uvms.err.rock]; 
uvms.xdot.rock = Saturate(uvms.xdot.rock, 1);

% horizental
uvms.xdot.ha = 1 * (0 - norm(uvms.v_rho_ha)); % norm(v_rho) IS THETA

% minimum altitude
uvms.xdot.ma = 10* (1 - uvms.a);

% landing
uvms.xdot.landing = 0.8* (0 - uvms.a);
