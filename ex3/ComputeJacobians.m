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

jlmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
jlmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];


pandaArm.Ste = [eye(3) zeros(3);  -skew(pandaArm.eTt(1:3,4)) eye(3)];
pandaArm.bJt  = pandaArm.Ste * pandaArm.bJe(:,1:7);


% Top three rows are angular velocities, bottom three linear velocities
 pandaArm.wJt  = pandaArm.bJt;
 pandaArm.wJt  = [pandaArm.wTb(1:3,1:3) zeros(3);zeros(3) pandaArm.wTb(1:3,1:3)]* pandaArm.bJt;

% if mission.phase == 2
%     pandaArm.wJo = ...;
% end
% 
% pandaArm.Jjl = ;
end
