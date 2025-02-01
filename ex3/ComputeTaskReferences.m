function [pandaArm] = ComputeTaskReferences(pandaArm,mission)

    % Compute minimum altitude reference ALWAYS
    % End effector minimum altitude for right
    pandaArm.min_dis = -pandaArm.wTt(3,4);
    pandaArm.xdot.min = 1* (0.20 - pandaArm.min_dis);
    pandaArm.xdot.min= [0 0 0 0 0 pandaArm.xdot.min]';

    % Compute joint limits task reference ALWAYS
    % Create a velocity away from the limits => move to the middle between jlmax and jlmin
    pandaArm.xdot.joints = zeros(7,1);
    for i = 1:7
        pandaArm.xdot.joints(i) = ((pandaArm.jlmin(i) + pandaArm.jlmax(i)) / 2) - pandaArm.q(i);
    end
    % Rigid Grasp Constraint
    pandaArm.xdot.grasp = [0 0 0 0 (0.2*(0.3 - pandaArm.q(7))) 0]';
    pandaArm.xdot.grasp = Saturate(pandaArm.xdot.grasp,0.2);

    switch mission.phase
        case 1
            % Tool position and orientation task reference
            [pandaArm.ang, pandaArm.lin] = CartError(pandaArm.wTg,pandaArm.wTt);
            pandaArm.xdot.tool = [pandaArm.ang;pandaArm.lin];
            pandaArm.xdot.tool = Saturate(pandaArm.xdot.tool,1);

        case 2
            % Object position and orientation task reference
            [pandaArm.ang, pandaArm.lin] = CartError(pandaArm.wTog,pandaArm.wTto);
            pandaArm.xdot.tool = [pandaArm.ang;pandaArm.lin];
            pandaArm.xdot.tool = Saturate(pandaArm.xdot.tool,0.2);

        case 3
            % Stop any motions
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            pandaArm.xdot.tool = zeros(6,1);
    end
end