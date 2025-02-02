function [pandaArm] = ComputeActivationFunctions(pandaArm, mission)

    % EQUALITY TASK ACTIVATION
    switch mission.phase
        case 1  % Reach the grasping point
            prev = mission.actions.go_to.tasks;
            current = mission.actions.go_to.tasks;
            
            % constraint
            pandaArm.Ap.con = 0;

        case 2 % Move the object holding it firmly
            prev = mission.actions.go_to.tasks;
            current = mission.actions.coop_manip.tasks;

            % constraint
            pandaArm.Ap.con = 1;
            
        case 3 % STOP any motion 
            prev = mission.actions.coop_manip.tasks;
            current = mission.actions.end_motion.tasks;

            % constraint
            pandaArm.Ap.con = 0;

    end

    % Move-To
    pandaArm.ArmR.A.tool = eye(6) *  ActionTransition("T", prev, current, mission.phase_time);
    pandaArm.ArmL.A.tool = eye(6) *  ActionTransition("T", prev, current, mission.phase_time);

    % Rigid Grasp Constraint
    pandaArm.ArmL.A.grasp = eye(6) * ActionTransition("G", prev, current, mission.phase_time);
    pandaArm.ArmR.A.grasp = eye(6) * ActionTransition("G", prev, current, mission.phase_time);

    % constraint
    pandaArm.A.con = eye(6) * pandaArm.Ap.con;

    % INEQUALITY TASK ACTIVATION
    % Minimum Altitude Task ( > 0.15m, 0.05m delta )
    pandaArm.ArmL.A.min = eye(6) * DecreasingBellShapedFunction(0.15, 0.20 , 0, 1, pandaArm.ArmL.min_dis) *  ActionTransition("MA", prev, current, mission.phase_time);
    pandaArm.ArmR.A.min = eye(6) * DecreasingBellShapedFunction(0.15, 0.20 , 0, 1, pandaArm.ArmR.min_dis) *  ActionTransition("MA", prev, current, mission.phase_time);

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
    pandaArm.ArmL.A.joints = pandaArm.ArmL.A.joints  *  ActionTransition("JL", prev, current, mission.phase_time);
    pandaArm.ArmR.A.joints = pandaArm.ArmR.A.joints  *  ActionTransition("JL", prev, current, mission.phase_time);

end
