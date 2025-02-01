function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

    % EQUALITY TASK ACTIVATION
    switch mission.phase
        case 1  % Reach the grasping point
            % Move-To
            pandaArm.Ap.tool = 1;
            % Rigid Grasp Constraint
            pandaArm.Ap.grasp = 0;

        case 2 % Move the object holding it firmly
            % Move-To
            pandaArm.Ap.tool = 1;
            % Rigid Grasp Constraint
            pandaArm.Ap.grasp = 1;

        case 3 % STOP any motion 
            pandaArm.Ap.tool = 1;
    end

    % Move-To
    pandaArm.A.tool = eye(6) * pandaArm.Ap.tool;
    % Rigid Grasp Constraint
    pandaArm.A.grasp = eye(6) * pandaArm.Ap.grasp;

    % INEQUALITY TASK ACTIVATION
    % Minimum Altitude Task ( > 0.15m, 0.05m delta )
    pandaArm.A.min = eye(6)*DecreasingBellShapedFunction(0.15, 0.20 , 0, 1, pandaArm.min_dis);

    % Joint Limits Task
    % Activation function: two combined sigmoids, which are at their maximum 
    % at the joint limits and approach zero between them    
    % Safety Task (inequality)
    % delta is 10% of max error
    pandaArm.joints=zeros(7);
    for i = 1:7
        pandaArm.A.joints(i,i) = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i) * 1.1, 0, 1, pandaArm.q(i)) ...
            + IncreasingBellShapedFunction(pandaArm.jlmax(i) * 0.9, pandaArm.jlmax(i), 0, 1, pandaArm.q(i));
    end
end