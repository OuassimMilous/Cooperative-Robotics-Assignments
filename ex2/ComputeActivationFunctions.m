function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
         pandaArm.ArmR.A.tool = eye(6);
         pandaArm.ArmL.A.tool = eye(6);

        % minimum altitude
         pandaArm.ArmL.A.min = zeros(6);
         if(pandaArm.ArmL.xdot.min(6)<0)
              pandaArm.ArmL.A.min = eye(6);
         end

         pandaArm.ArmR.A.min = zeros(6);
          if(pandaArm.ArmL.xdot.min(6)<0)
              pandaArm.ArmR.A.min = eye(6);
          end


        % limits joints
        % max        pandaArm.ArmL.A.joints_min=ones(6);
        pandaArm.ArmR.A.joints_min=ones(6);

        pandaArm.ArmL.A.joints_max=ones(6);
        pandaArm.ArmR.A.joints_max=ones(6);

        % min
        pandaArm.ArmL.A.joints_min=ones(6);
        pandaArm.ArmR.A.joints_min=ones(6);

    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        
        % Move-To
        
    case 3 % STOP any motion 
        
end
% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
% pandaArm.A.ma = ...;

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
% pandaArm.A.jl = ...;

end
