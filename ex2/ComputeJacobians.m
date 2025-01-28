function [pandaArms] = ComputeJacobians(pandaArms,mission)
% compute the relevant Jacobians here
% joint limits
% tool-frame position control (to do)
% initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, 
% -0.0305686, 1.53975, 0.753872] ) 
%
% remember: the control vector is:
% [q_dot] 
% [qdot_1, qdot_2, ..., qdot_7]
%
% therefore all task jacobians should be of dimensions
% m x 14
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]

% Left Arm base to ee Jacobian
pandaArms.ArmL.bJe = geometricJacobian(pandaArms.ArmL.franka, ...
    [pandaArms.ArmL.q',0,0],'panda_link7');%DO NOT EDIT
% Right Arm base to ee Jacobian
pandaArms.ArmR.bJe = geometricJacobian(pandaArms.ArmR.franka, ...
    [pandaArms.ArmR.q',0,0],'panda_link7');%DO NOT EDIT

pandaArms.ArmL.Ste = [eye(3) zeros(3);  -skew(pandaArms.ArmL.eTt(1:3,4)) eye(3)];
pandaArms.ArmL.bJt  = pandaArms.ArmL.Ste * pandaArms.ArmL.bJe(:,1:7);

pandaArms.ArmR.Ste = [eye(3) zeros(3);  -skew(pandaArms.ArmR.eTt(1:3,4)) eye(3)];
pandaArms.ArmR.bJt  = pandaArms.ArmR.Ste * pandaArms.ArmR.bJe(:,1:7);

% Top three rows are angular velocities, bottom three linear velocities
pandaArms.ArmL.wJt  = pandaArms.ArmL.bJt;
pandaArms.ArmR.wJt  = [pandaArms.ArmR.wTb(1:3,1:3) zeros(3);zeros(3) pandaArms.ArmR.wTb(1:3,1:3)]* pandaArms.ArmR.bJt;
% display(pandaArms.ArmL.bJe)
% display(pandaArms.ArmR.wJt)

pandaArms.ArmL.Jma = [zeros(5,14);zeros(1,5), 1 ,0 , zeros(1,7)];
pandaArms.ArmR.Jma = [zeros(5,14);zeros(1,7) zeros(1,5), 1 ,0 ];

% limits joints
% min
pandaArms.ArmL.bJm_min = zeros(6,14);
for i = 1:length(pandaArms.ArmL.joints_dis_min)
    pandaArms.ArmL.bJm_min(3,i) = pandaArms.ArmL.joints_dis_min(i);
end

pandaArms.ArmR.bJm_min = zeros(6,14);
for i = 1:length(pandaArms.ArmR.joints_dis_min)
    pandaArms.ArmR.bJm_min(3,i) = pandaArms.ArmR.joints_dis_min(i);
end

% max
pandaArms.ArmL.bJm_max = zeros(6,14);
for i = 1:length(pandaArms.ArmL.joints_dis_max)
    pandaArms.ArmL.bJm_max(3,i) = pandaArms.ArmL.joints_dis_max(i);
end
pandaArms.ArmR.bJm_max = zeros(6,14);
for i = 1:length(pandaArms.ArmR.joints_dis_max)
    pandaArms.ArmR.bJm_max(3,i) = pandaArms.ArmR.joints_dis_max(i);
end



% display(pandaArms.ArmL.bJm_max)


if (mission.phase == 2)
    % pandaArms.ArmL.wJo = ...; 
    % pandaArms.ArmR.wJo = ...;
    % Grasping 
    pandaArms.ArmL.bJt_grasp = zeros(6,14);
    pandaArms.ArmL.bJt_grasp(:,7) = [0 0 0 0 1 0];
    
    
    pandaArms.ArmR.bJt_grasp = zeros(6,14);
    pandaArms.ArmR.bJt_grasp(:,14) = [0 0 0 0 1 0];


% Common Jacobians
% pandaArms.Jjl = ...;

end