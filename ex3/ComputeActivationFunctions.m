function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
         pandaArm.A.tool = eye(6);
        
    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        pandaArm.A.grasp=eye(6);
        % Move-To
         pandaArm.A.tool = eye(6);

    case 3 % STOP any motion 
        
end
% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
% pandaArm.A.ma = ...;
 pandaArm.A.min = zeros(6);
 if(pandaArm.xdot.min(6)<0)
      pandaArm.A.min = eye(6);
 end

% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
% pandaArm.A.jl = ...;
pandaArm.A.joints=eye(6);
    
end