function [pandaArm] = ComputeTaskReferences(pandaArm,mission)

% % Compute minimum altitude reference ALWAYS
% pandaArm.xdot.alt = ...;
% % take the smallest value, that is what matters most
% % pandaArm.min_alt = min(alt_L, alt_R);



% End effector minimum altitude for right
pandaArm.min_dis = [pandaArm.wTt(3,4)];
pandaArm.xdot.min = -1* (0.15- norm(pandaArm.min_dis));
pandaArm.xdot.min= [0 0 0 0 0 pandaArm.xdot.min]';

% % Compute joint limits task reference ALWAYS
% % Create a velocity away from the limits => move to the middle between jlmax and jlmin
% pandaArm.xdot.jl = ...;
    
% joint limits
pandaArm.xdot.joints = zeros(7,1);   
switch mission.phase
    case 1
        % Tool position and orientation task reference
        % [ang, lin] = CartError(inv(pandaArm.wTb)*pandaArm.wTg,inv(pandaArm.wTb)*pandaArm.wTt);
        [ang, lin] = CartError(pandaArm.wTg,pandaArm.wTt);

        pandaArm.xdot.tool = [ang;lin];
        % limit the requested velocities...
        pandaArm.xdot.tool(1:3) = Saturate(pandaArm.xdot.tool(1:3),1);
        pandaArm.xdot.tool(4:6) = Saturate(pandaArm.xdot.tool(4:6),1);

    case 2
        % Rigid Grasp Constraint
        pandaArm.xdot.grasp = [0 0 0 0 (0.2*(0.3 - pandaArm.q(7))) 0]';
        % limit the requested velocities...
        pandaArm.xdot.grasp(1:3) = Saturate(pandaArm.xdot.grasp(1:3),0.2);
        pandaArm.xdot.grasp(4:6) = Saturate(pandaArm.xdot.grasp(4:6),0.2);
        % Object position and orientation task reference

        [ang, lin] = CartError(pandaArm.wTog,pandaArm.wTto);
        pandaArm.xdot.tool = [ang;lin];
        % display( [ang, lin])
        % limit the requested velocities...
        pandaArm.xdot.tool(1:3) = Saturate(pandaArm.xdot.tool(1:3),0.2);
        pandaArm.xdot.tool(4:6) = Saturate(pandaArm.xdot.tool(4:6),0.2);

    case 3
        % Stop any motions
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        pandaArm.xdot.tool(1:3) = zeros(3,1);
        pandaArm.xdot.tool(4:6) = zeros(3,1);
end


