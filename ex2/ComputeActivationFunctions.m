function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
         pandaArm.ArmR.A.tool = eye(6);
         pandaArm.ArmL.A.tool = eye(6);

        % Rigid Grasp Constraint
        pandaArm.ArmL.A.grasp=zeros(6);
        pandaArm.ArmR.A.grasp=zeros(6);

    case 2 % Move the object holding it firmly

         % Move-To
         pandaArm.ArmR.A.tool = eye(6);
         pandaArm.ArmL.A.tool = eye(6);

         % constraint
         pandaArm.A.con = eye(6);

        % Rigid Grasp Constraint
        pandaArm.ArmL.A.grasp=eye(6);
        pandaArm.ArmR.A.grasp=eye(6);
        
    case 3 % STOP any motion 

         % Move-To
         pandaArm.ArmR.A.tool = zeros(6);
         pandaArm.ArmL.A.tool = zeros(6);

         % constraint
         pandaArm.A.con = zeros(6);

        % Rigid Grasp Constraint
        pandaArm.ArmL.A.grasp=zeros(6);
        pandaArm.ArmR.A.grasp=zeros(6);

end
% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
% pandaArm.A.ma = ...;

% minimum altitude
% pandaArm.ArmL.A.min = zeros(6);
% if(pandaArm.ArmL.xdot.min(6)<0)
%         pandaArm.ArmL.A.min = eye(6);
% end
pandaArm.ArmL.A.ma = DecreasingBellShapedFunction(0.15, 0.20 , 0, 1, pandaArm.ArmL.min_dis);


% pandaArm.ArmR.A.min = zeros(6);
%     if(pandaArm.ArmL.xdot.min(6)<0)
%         pandaArm.ArmR.A.min = eye(6);
%     end    
pandaArm.ArmL.A.ma = DecreasingBellShapedFunction(0.15, 0.20 , 0, 1, pandaArm.ArmR.min_dis);
% limits joints
% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
% pandaArm.A.jl = ...;
pandaArm.ArmL.A.joints=zeros(14);
pandaArm.ArmR.A.joints=zeros(14);
for i = 1:7
    pandaArm.ArmL.A.joints(i,i) = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i) * 1.1, 0, 1, pandaArm.ArmL.q(i)) ...
        + IncreasingBellShapedFunction(pandaArm.jlmax(i) * 0.9, pandaArm.jlmax(i), 0, 1, pandaArm.ArmL.q(i));

    pandaArm.ArmR.A.joints(i+7, i+7) = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i) * 1.1, 0, 1, pandaArm.ArmR.q(i)) ...
        + IncreasingBellShapedFunction(pandaArm.jlmax(i) * 0.9, pandaArm.jlmax(i), 0, 1, pandaArm.ArmR.q(i));
end


end
