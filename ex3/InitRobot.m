function [pandaArm] = InitRobot(model,wTb)

    %% DO NOT CHANGE FROM HERE ...
    % Init two field of the main structure pandaArm containing the two robot
    % model
    pandaArm = model;
    % Init robot basic informations (q_init, transformation matrices ...)
    pandaArm.q = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';%check rigid body tree DOCUMENTATION
    pandaArm.q_dot = [0 0 0 0 0 0 0]';
    pandaArm.alt = 0.20;

    pandaArm.bTe = getTransform(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');
    pandaArm.wTb = wTb;
    pandaArm.wTe = pandaArm.wTb*pandaArm.bTe;

    % joint limits corresponding to the actual Panda by Franka arm configuration
    pandaArm.jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
    pandaArm.jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];


    % Init relevance Jacobians
    pandaArm.bJe = eye(6,7);

    %% ... TO HERE
    pandaArm.Ste = zeros(6);

    pandaArm.ang = zeros(3,1);
    pandaArm.lin = zeros(3,1);

    % move to
    pandaArm.xdot.tool = zeros(6,1);
    pandaArm.A.tool = zeros(6);
    pandaArm.Ap.tool = 0;
    pandaArm.Jma = zeros(6,7);

    % minimalt
    pandaArm.xdot.min = zeros(6);
    pandaArm.min_dis = 0;
    pandaArm.A.min = zeros(6,1);
    pandaArm.Ap.min = 0;
    pandaArm.bJt = zeros(6,7);


    % Fixed joints
    pandaArm.xdot.joints = zeros(6,1);
    pandaArm.A.joints = zeros(6);
    pandaArm.Ap.joints = 0;
    pandaArm.bJm = zeros(6,7);


    %% PHASE 2

    pandaArm.tTo = zeros(4);
    pandaArm.wTo = zeros(4);

    % constraint task
    pandaArm.A.con = zeros(6);
    pandaArm.Ap.con = 0;
    pandaArm.xdot.con = zeros(6,1);
    pandaArm.Jjl = zeros(6,14);

    % move to 2
    pandaArm.ArmL.wJo= [];

    % Grasping
    pandaArm.xdot.grasp = zeros(6,1);
    pandaArm.Ap.grasp = 0;
    pandaArm.A.grasp = zeros(6);
    pandaArm.bJt_grasp = zeros(6,7);

end