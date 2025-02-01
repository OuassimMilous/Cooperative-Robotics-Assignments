function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)

    % EQUALITY TASK ACTIVATION
    switch mission.phase
        case 1  % Reach the grasping point
            % Move-To
            pandaArm.ArmR.Ap.tool = 1;
            pandaArm.ArmL.Ap.tool = 1;

            % Rigid Grasp Constraint
            pandaArm.ArmL.Ap.grasp = 0;
            pandaArm.ArmR.Ap.grasp = 0;

            % constraint
            pandaArm.Ap.con = 0;

        case 2 % Move the object holding it firmly

            % Move-To
            pandaArm.ArmR.Ap.tool = 1;
            pandaArm.ArmL.Ap.tool = 1;

            % constraint
            pandaArm.Ap.con = 1;

            % Rigid Grasp Constraint
            pandaArm.ArmL.Ap.grasp=eye(6);
            pandaArm.ArmR.Ap.grasp=eye(6);
            
        case 3 % STOP any motion 

            % Move-To
            pandaArm.ArmR.Ap.tool = 0;
            pandaArm.ArmL.Ap.tool = 0;

            % constraint
            pandaArm.Ap.con = 0;

            % Rigid Grasp Constraint
            pandaArm.ArmL.Ap.grasp = 0;
            pandaArm.ArmR.Ap.grasp = 0;

    end

    % Move-To
    pandaArm.ArmR.A.tool = eye(6) * pandaArm.ArmR.Ap.tool;
    pandaArm.ArmL.A.tool = eye(6) * pandaArm.ArmL.Ap.tool;

    % Rigid Grasp Constraint
    pandaArm.ArmL.A.grasp = zeros(6) * pandaArm.ArmL.Ap.grasp;
    pandaArm.ArmR.A.grasp = zeros(6) * pandaArm.ArmR.Ap.grasp;

    % constraint
    pandaArm.A.con = eye(6) * pandaArm.Ap.con;

    % INEQUALITY TASK ACTIVATION

    % Minimum Altitude Task ( > 0.15m, 0.05m delta )

    % minimum altitude
    pandaArm.ArmL.A.min = eye(6) * DecreasingBellShapedFunction(0.15, 0.20 , 0, 1, pandaArm.ArmL.min_dis);
    pandaArm.ArmR.A.min = eye(6) * DecreasingBellShapedFunction(0.15, 0.20 , 0, 1, pandaArm.ArmR.min_dis);

    % limits joints
    % Joint Limits Task
    % Activation function: two combined sigmoids, which are at their maximum 
    % at the joint limits and approach zero between them    
    % Safety Task (inequality)
    % delta is 10% of max error
    pandaArm.ArmL.A.joints=zeros(14);
    pandaArm.ArmR.A.joints=zeros(14);
    for i = 1:7
        pandaArm.ArmL.A.joints(i,i) = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i) * 1.1, 0, 1, pandaArm.ArmL.q(i)) ...
            + IncreasingBellShapedFunction(pandaArm.jlmax(i) * 0.9, pandaArm.jlmax(i), 0, 1, pandaArm.ArmL.q(i));

        pandaArm.ArmR.A.joints(i+7, i+7) = DecreasingBellShapedFunction(pandaArm.jlmin(i), pandaArm.jlmin(i) * 1.1, 0, 1, pandaArm.ArmR.q(i)) ...
            + IncreasingBellShapedFunction(pandaArm.jlmax(i) * 0.9, pandaArm.jlmax(i), 0, 1, pandaArm.ArmR.q(i));
    end

end
