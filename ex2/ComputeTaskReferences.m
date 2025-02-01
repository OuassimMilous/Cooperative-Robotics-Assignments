function [pandaArm] = ComputeTaskReferences(pandaArm,mission)
% Compute distance between tools for plotting
pandaArm.dist_tools = norm(pandaArm.ArmL.wTt(1:3, 4) - pandaArm.ArmR.wTt(1:3, 4));
% Compute minimum altitude reference ALWAYS

% End effector minimum altitude for left
pandaArm.ArmL.min_dis = -pandaArm.ArmL.wTt(3,4);
pandaArm.ArmL.xdot.min = 1 * (0.20 - pandaArm.ArmL.min_dis);
pandaArm.ArmL.xdot.min = [0 0 0 0 0 pandaArm.ArmL.xdot.min]';

% End effector minimum altitude for right
pandaArm.ArmR.min_dis = -pandaArm.ArmR.wTt(3,4);
pandaArm.ArmR.xdot.min = 1* (0.20- pandaArm.ArmR.min_dis);
pandaArm.ArmR.xdot.min= [0 0 0 0 0 pandaArm.ArmR.xdot.min]';


% Compute joint limits task reference ALWAYS
% Create a velocity away from the limits => move to the middle between jlmax and jlmin
pandaArm.ArmL.xdot.joints = zeros(14,1);
pandaArm.ArmR.xdot.joints = zeros(14,1);       
for i = 1:7
    pandaArm.ArmL.xdot.joints(i) = ((pandaArm.jlmin(i) + pandaArm.jlmax(i)) / 2) - pandaArm.ArmL.q;
    pandaArm.ArmR.xdot.joints(i+7) = ((pandaArm.jlmin(i) + pandaArm.jlmax(i)) / 2) - pandaArm.ArmR.q;
end

    switch mission.phase
        case 1
            % LEFT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            [pandaArm.ArmL.ang_l, pandaArm.ArmL.lin_l] = CartError(pandaArm.ArmL.wTg,pandaArm.ArmL.wTt);

            pandaArm.ArmL.xdot.tool = [pandaArm.ArmL.ang_l;pandaArm.ArmL.lin_l];
            pandaArm.ArmL.xdot.tool = Saturate(pandaArm.ArmL.xdot.tool,1);

            % RIGHT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            [pandaArm.ArmR.ang_r, pandaArm.ArmR.lin_r] = CartError(pandaArm.ArmR.wTg,pandaArm.ArmR.wTt);
        
            pandaArm.ArmR.xdot.tool = [pandaArm.ArmR.ang_r;pandaArm.ArmR.lin_r];
            pandaArm.ArmR.xdot.tool = Saturate(pandaArm.ArmR.xdot.tool,1);

        case 2
            % Perform the rigid grasp of the object and move it

            % COMMON
            % -----------------------------------------------------------------
            % con
            pandaArm.xdot.con = zeros(6,1);

            % LEFT ARM
            % -----------------------------------------------------------------        
            % Object position and orientation task reference

            % move2
            [pandaArm.ArmL.ang_l, pandaArm.ArmL.lin_l] = CartError(pandaArm.wTog,pandaArm.ArmL.wTo);

            pandaArm.ArmL.xdot.tool = 0.35*[pandaArm.ArmL.ang_l;pandaArm.ArmL.lin_l];
            pandaArm.ArmL.xdot.tool = Saturate(pandaArm.ArmL.xdot.tool,0.35);

            % grasp
            pandaArm.ArmL.xdot.grasp = [0 0 0 0 (0.35*(0.3 - pandaArm.ArmL.q(7))) 0]';
            pandaArm.ArmL.xdot.grasp = Saturate(pandaArm.ArmL.xdot.grasp,0.35);

            % RIGHT ARM
            % -----------------------------------------------------------------

            % move2
            [pandaArm.ArmR.ang_l, pandaArm.ArmR.lin_l] = CartError(pandaArm.wTog,pandaArm.ArmR.wTo);

            pandaArm.ArmR.xdot.tool = 0.35*[pandaArm.ArmR.ang_l;pandaArm.ArmR.lin_l];
            pandaArm.ArmR.xdot.tool = Saturate(pandaArm.ArmR.xdot.tool,0.35);

            % grasp
            pandaArm.ArmR.xdot.grasp = [0 0 0 0 (0.35*(0.03 - pandaArm.ArmR.q(7))) 0]';
            pandaArm.ArmR.xdot.grasp = Saturate(pandaArm.ArmR.xdot.grasp,0.35);


        case 3
            % Stop any motions
            % LEFT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            pandaArm.ArmL.xdot.tool = zeros(6,1);

            % RIGHT ARM
            % -----------------------------------------------------------------
            % Tool position and orientation task reference
            pandaArm.ArmR.xdot.tool = zeros(6,1);

    end
end