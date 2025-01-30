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

% uvms.vTr =inv(uvms.wTv)*uvms.wTr;
% [ang_closer, lin_closer] = CartError(uvms.vTr , uvms.vTt);
% uvms.xdot.closer = 1 * [ang_closer; lin_closer];
% uvms.xdot.closer(1:3) = Saturate(uvms.xdot.closer(1:3), 1);
% uvms.xdot.closer(4:6) = Saturate(uvms.xdot.closer(4:6), 1);

[ang_closer, lin_closer] = CartError(uvms.wTg , uvms.wTb);
uvms.xdot.closer = [lin_closer(1:2);0];
uvms.xdot.closer = Saturate(uvms.xdot.closer, 1);
% display(uvms.xdot.closer)
% display([lin_closer])
% [ang_closer, lin_closer] = CartError(inv(uvms.vTb),inv(uvms.vTr));
% display([lin_closer()])
% uvms.xdot.closer = [-1*lin_closer(1:2);zeros(4,1)];
% uvms.xdot.closer = Saturate(uvms.xdot.closer, 1);


%rock
[ang_rock, lin_rock] = CartError(uvms.wTg , uvms.wTb);
uvms.xdot.rock = 5*ang_rock();
display(ang_rock(3))

% disp("uvms.wTv")
% disp(uvms.wTv)

uvms.xdot.rock = Saturate(uvms.xdot.rock, 5);

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
