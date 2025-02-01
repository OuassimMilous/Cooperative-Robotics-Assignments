function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

% EQUALITY TASK ACTIVATION
switch mission.phase
    case 1  % Reach the grasping point
        % Move-To
         pandaArm.A.tool = eye(6);
         pandaArm.A.grasp = zeros(6);

    case 2 % Move the object holding it firmly
        % Rigid Grasp Constraint
        pandaArm.A.grasp = eye(6);
        % Move-To
         pandaArm.A.tool = eye(6);

    case 3 % STOP any motion 
        
end
% INEQUALITY TASK ACTIVATION
% Minimum Altitude Task ( > 0.15m, 0.05m delta )
% pandaArm.A.ma = ...;
pandaArm.A.ma = DecreasingBellShapedFunction(0.15, 0.20 , 0, 1, pandaArm.min_dis);


% Joint Limits Task
% Activation function: two combined sigmoids, which are at their maximum 
% at the joint limits and approach zero between them    
% Safety Task (inequality)
% delta is 10% of max error
% pandaArm.A.jl = ...;
pandaArm.joints=zeros(7);
for i = 1:7
    pandaArm.A.joints(i,i) = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i) * 1.1, 0, 1, pandaArm.q(i)) ...
        + IncreasingBellShapedFunction(pandaArm.jlmax(i) * 0.9, pandaArm.jlmax(i), 0, 1, pandaArm.q(i));
end
display("activation function Left:");
display(pandaArm.A.joints);
display("activation function right:");
display(pandaArm.A.joints);

end