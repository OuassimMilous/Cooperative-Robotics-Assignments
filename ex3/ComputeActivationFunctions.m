function [pandaArm] = ComputeActivationFunctions(pandaArm,mission)

    % EQUALITY TASK ACTIVATION
    switch mission.phase
        case 1  % Reach the grasping point

            prev = mission.actions.go_to.tasks;
            current = mission.actions.go_to.tasks;

            % cooperation
            pandaArm.Ap.coop = 0;
        case 2 % Move the object holding it firmly
            prev = mission.actions.go_to.tasks;
            current = mission.actions.coop_manip.tasks;

            % cooperation
            pandaArm.Ap.coop = 1;

        case 3 % STOP any motion 
            prev = mission.actions.coop_manip.tasks;
            current = mission.actions.end_motion.tasks;

            % cooperation
            pandaArm.Ap.coop = 0;
            end

    % Move-To
    pandaArm.A.tool = eye(6) * ActionTransition("T", prev, current, mission.phase_time);
    % Rigid Grasp Constraint
    pandaArm.A.grasp = eye(6) *  ActionTransition("G", prev, current, mission.phase_time);

    % INEQUALITY TASK ACTIVATION
    % Minimum Altitude Task ( > 0.15m, 0.05m delta )
    pandaArm.A.min = eye(6)*DecreasingBellShapedFunction(0.15, 0.20 , 0, 1, pandaArm.min_dis) * ActionTransition("MA", prev, current, mission.phase_time);;

    % coop
    pandaArm.A.coop = eye(6) * pandaArm.Ap.coop;

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
    pandaArm.A.joints = pandaArm.A.joints  *  ActionTransition("JL", prev, current, mission.phase_time);

end