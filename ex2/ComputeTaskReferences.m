function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Compute distance between tools for plotting
pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));
% Compute minimum altitude reference ALWAYS

% pandaArm.ArmL.xdot.alt = ...;
% pandaArm.ArmR.xdot.alt = ...;

% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
% 
% pandaArm.ArmL.xdot.jl = ...;
% pandaArm.ArmR.xdot.jl = ...;

switch mission.phase
    case 1
        % LEFT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang_l, lin_l] = CartError(pandaArm.ArmL.wTt,pandaArm.ArmL.wTg);
       
        pandaArm.ArmL.xdot.tool = -0.2*[ang_l;lin_l];
        % limit the requested velocities...
        pandaArm.ArmL.xdot.tool(1:3) = Saturate(pandaArm.ArmL.xdot.tool(1:3),0.2);
        pandaArm.ArmL.xdot.tool(4:6) = Saturate(pandaArm.ArmL.xdot.tool(4:6),0.2);

        % RIGHT ARM
        % -----------------------------------------------------------------
        % Tool position and orientation task reference
        [ang_r, lin_r] = CartError(pandaArm.ArmR.wTt,pandaArm.ArmR.wTg);
       
        pandaArm.ArmR.xdot.tool = -0.2*[ang_r;lin_r];
        % limit the requested velocities...
        pandaArm.ArmR.xdot.tool(1:3) = Saturate(pandaArm.ArmR.xdot.tool(1:3),0.2);
        pandaArm.ArmR.xdot.tool(4:6) = Saturate(pandaArm.ArmR.xdot.tool(4:6),0.2);
    % case 2
    %     % Perform the rigid grasp of the object and move it
    % 
    %     % COMMON
    %     % -----------------------------------------------------------------
    %     % Rigid Grasp Constraint
    %     pandaArm.xdot.rc = ...;
    % 
    %     % LEFT ARM
    %     % -----------------------------------------------------------------        
    %     % Object position and orientation task reference
    %     [ang, lin] = CartError();
    %     pandaArm.ArmL.xdot.tool = ...;
    %     % limit the requested velocities...
    %     pandaArm.ArmL.xdot.tool(1:3) = Saturate();
    %     pandaArm.ArmL.xdot.tool(4:6) = Saturate();
    % 
    %     % RIGHT ARM
    %     % -----------------------------------------------------------------
    %     % Object position and orientation task reference
    %     [ang, lin] = CartError();
    %     pandaArm.ArmR.xdot.tool = ...;
    %     % limit the requested velocities...
    %     pandaArm.ArmR.xdot.tool(1:3) = Saturate();
    %     pandaArm.ArmR.xdot.tool(4:6) = Saturate();
    % case 3
    %     % Stop any motions
    %     % LEFT ARM
    %     % -----------------------------------------------------------------
    %     % Tool position and orientation task reference
    %     pandaArm.ArmL.xdot.tool(1:3) = ...;
    %     pandaArm.ArmL.xdot.tool(4:6) = ...;
    % 
    %     % RIGHT ARM
    %     % -----------------------------------------------------------------
    %     % Tool position and orientation task reference
    %     pandaArm.ArmR.xdot.tool(1:3) = ...;
    %     pandaArm.ArmR.xdot.tool(4:6) = ...;
end


