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


jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

% jlmin = 9*ones(7,1);
% jlmax = -9*ones(7,1);



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


for i = 1:7
    if (pandaArms.ArmL.q(i)<jlmax(i)||pandaArms.ArmL.q(i)>jlmin(i))
        pandaArms.ArmL.joints_switch(i,i) = 0;
    else
        pandaArms.ArmL.joints_switch(i,i) = 1;
    end
end

pandaArms.ArmL.bJm = [pandaArms.ArmL.bJe(1:6,1:7) * pandaArms.ArmL.joints_switch zeros(6,7)];

% display([jlmax, jlmin])
% display([pandaArms.ArmL.q])

for i = 1:7
    if (pandaArms.ArmR.q(i)<jlmax(i)||pandaArms.ArmR.q(i)>jlmin(i))
        pandaArms.ArmR.joints_switch(i,i) = 0;
    else
        pandaArms.ArmR.joints_switch(i,i) = 1;
    end
end

pandaArms.ArmR.bJm = [zeros(6,7) pandaArms.ArmR.bJe(1:6,1:7) * pandaArms.ArmR.joints_switch];

% display(pandaArms.ArmL.bJm)





if (mission.phase == 2)

    pandaArms.ArmL.wJo = pandaArms.ArmL.Jma;
    pandaArms.ArmR.wJo = pandaArms.ArmL.Jma;
    % Grasping 
    pandaArms.ArmL.bJt_grasp = zeros(6,14);
    pandaArms.ArmL.bJt_grasp(:,7) = [0 0 0 0 1 0];
    
    
    pandaArms.ArmR.bJt_grasp = zeros(6,14);
    pandaArms.ArmR.bJt_grasp(:,14) = [0 0 0 0 1 0];

% con
% Common Jacobians
pandaArms.Jjl = [pandaArms.ArmL.wJt  -pandaArms.ArmR.wJt ];

end