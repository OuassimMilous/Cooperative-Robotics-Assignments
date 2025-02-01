function [pandaArm] = ComputeJacobians(pandaArm,mission)
    % compute the relevant Jacobians here
    % joint limits
    % tool-frame position control (to do)
    % initial arm posture ( [0.0167305, -0.762614, -0.0207622, -2.34352, -0.0305686, 1.53975, 0.753872] ) 
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

    % Jacobian from base to end-effector
    pandaArm.bJe = geometricJacobian(pandaArm.franka,[pandaArm.q',0,0],'panda_link7');
    % THE JACOBIAN bJe has dimension 6x9 (the matlab model include the joint
    % of the gripper). YOU MUST RESIZE THE MATRIX IN ORDER TO CONTROL ONLY THE
    % 7 JOINTS OF THE ROBOTIC ARM. 

    % tool
    pandaArm.wJt  = [pandaArm.wTb(1:3,1:3) zeros(3);zeros(3) pandaArm.wTb(1:3,1:3)]* pandaArm.bJe(:,1:7);
    pandaArm.Ste = [eye(3) zeros(3);  -skew(pandaArm.wTe(1:3,1:3)*pandaArm.eTt(1:3,4)) eye(3)];
    pandaArm.wJt  = pandaArm.Ste * pandaArm.wJt;

    if mission.phase == 2
        pandaArm.Ste = [eye(3) zeros(3);  -skew(pandaArm.wTog(1:3,4)) eye(3)];
        pandaArm.wJo = pandaArm.Ste *pandaArm.wJt;
    end

    % minimum altitude
    pandaArm.Jma = [zeros(5,7);0,0,0,0,0, 1 ,0];

    % limits joints
    pandaArm.bJm = eye(7);

    % Grasping 
    pandaArm.bJt_grasp(:,7) = [0 0 0 0 1 0];
end
