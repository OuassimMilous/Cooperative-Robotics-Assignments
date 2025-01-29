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
pandaArm.Jjl = [];

%% ... TO HERE
pandaArm.Ste = zeros(6);
pandaArm.wTb; %fixed transformation word -> base left

% move to
pandaArm.xdot.tool = zeros(6);
pandaArm.A.tool=zeros(6);
pandaArm.Jma = [];


% minimalt
pandaArm.xdot.min = zeros(6);
pandaArm.min_dis = 0;
pandaArm.A.min=zeros(6);
pandaArm.bJt = [];


% Fixed joints
pandaArm.xdot.joints = zeros(6);
pandaArm.joints_switch = zeros(7);
pandaArm.A.joints=zeros(6);
pandaArm.bJm = [];


%% PHASE 2

% pandaArms.wTog = 
% constraint task
pandaArm.A.con=zeros(6);
pandaArm.xdot.con = zeros(6,1);
pandaArm.Jjl = zeros(6,14);


% move to 2
pandaArm.ArmL.wJo= [];
% Grasping
pandaArm.xdot.grasp = zeros(6,1);
pandaArm.A.grasp=zeros(6);
pandaArm.bJt_grasp = zeros(6,7);


end

