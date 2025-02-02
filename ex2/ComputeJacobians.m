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

    % move to
    pandaArms.ArmL.Stt = [eye(3) zeros(3);  -skew(pandaArms.ArmL.wTe(1:3,1:3)*pandaArms.ArmL.eTt(1:3,4)) eye(3)];
    pandaArms.ArmL.wJe  = [pandaArms.ArmL.wTb(1:3,1:3) zeros(3);zeros(3) pandaArms.ArmL.wTb(1:3,1:3)]* pandaArms.ArmL.bJe(:,1:7);
    pandaArms.ArmL.wJt  = pandaArms.ArmL.Stt * pandaArms.ArmL.wJe;

    pandaArms.ArmR.Stt = [eye(3) zeros(3);  -skew(pandaArms.ArmR.wTe(1:3,1:3)*pandaArms.ArmR.eTt(1:3,4)) eye(3)];
    pandaArms.ArmR.wJe  = [pandaArms.ArmR.wTb(1:3,1:3) zeros(3);zeros(3) pandaArms.ArmR.wTb(1:3,1:3)]* pandaArms.ArmR.bJe(:,1:7);
    pandaArms.ArmR.wJt  = pandaArms.ArmR.Stt * pandaArms.ArmR.wJe;
   
    if (mission.phase == 2)
        pandaArms.ArmL.Sto = [eye(3) zeros(3);  -skew(pandaArms.wTog(1:3,4)) eye(3)];
        pandaArms.ArmR.Sto = [eye(3) zeros(3);  -skew(pandaArms.wTog(1:3,4)) eye(3)];

        pandaArms.ArmL.wJo = pandaArms.ArmR.Sto * pandaArms.ArmL.wJt;
        pandaArms.ArmR.wJo = pandaArms.ArmR.Sto * pandaArms.ArmR.wJt;

    end

    % minimum altitude
    pandaArms.ArmL.Jma = [zeros(5,14);zeros(1,5), 1 ,0 , zeros(1,7)];
    pandaArms.ArmR.Jma = [zeros(5,14);zeros(1,7) zeros(1,5), 1 ,0 ];

    % joint limits
    pandaArms.ArmL.bJm = eye(14);
    pandaArms.ArmR.bJm = eye(14);

    % Grasping 
    pandaArms.ArmL.bJt_grasp(:,7) = [0 0 0 0 1 0];
    pandaArms.ArmR.bJt_grasp(:,14) = [0 0 0 0 1 0];
    
    % con
    pandaArms.con = [pandaArms.ArmL.wJt  -pandaArms.ArmR.wJt ];
end