 function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];
% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe; 
% 
% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];
% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

% NEW LINE
%HERE WE CAN TWO POSSIBILITY, DRIVE THE DISTANCE TO ZERO OR CONSIDER THE POINT (ATTACH TO THE VEHICLE FRAME) GOES TO THAT POINT
%JACOBIAN SIGN AND REFRENCE RATE WOULD BE CHANGE
%WE ARE HERE SECOND ONE (IT ALSO MEAN VELOCITY CONTROL), JACOBIAN SIZE -
%(m,n) m => NUMBER OF TASK AND n => SIZE OF CONTROL VECTOR (WHICH IS Y DOT, MENTIONED ABOVE)

%VECHILCE POSITION AND ORIENTATION JACOBIAN MATRIX PROJECTED ON WORLD FRAME. 
uvms.Jv_l = [zeros(3,7) uvms. wTv(1:3, 1:3) zeros(3)]; %LINEAR PART
uvms.Jv_a =  [zeros(3,7)    zeros(3)    uvms. wTv(1:3, 1:3)];   %ANGULAR PART


% display(uvms. wTr)

%JACOBIAN DEFINE FOR THE VEHICLE POSITION FOR THE ROCK TASK
uvms.Jv_r =  [zeros(3,7)    zeros(3)    uvms. wTr(1:3, 1:3)];

%IF IT WOULD BE IN VEHICLE FRAME THEN WE NEED TO PUT IDENTITY MATRIX, PG - 23 (IN NOTES)


%COMPUTE TASK REFRENCES - HORIZONTAL ALTITUDE
%HORIZONTAL ALTITUDE AXIS OF VEHICLE FRAME AND WORLD FRAME
v_kv = [0 0 1]';
w_kw = [0 0 1]';

v_kw = (uvms.vTw(1:3,1:3) * w_kw); %Z AXIS OF <w> PROJECTED ON <v>

%INVERSE ANGLE-AXIS IMPLEMENTATION (GIVES YOU AXIS OF ROTATION SCALED BY AN ANGLE)
uvms.v_rho_ha = ReducedVersorLemma(v_kw, v_kv);
uvms.v_n_ha = uvms.v_rho_ha ./ norm(uvms.v_rho_ha); %ONLY AXIS OF ROTATION

%VEHICLE HORIZONTAL 
uvms.Jha = [zeros(1,7) zeros(1,3) uvms.v_n_ha'];

%VECHILE MINIMUM ALTITUDE
v_d = [0; 0; uvms.sensorDistance];
uvms.a = v_kw' * v_d; %THIS IS SCALAR DISTANCE, SENSOR DISTANCE COMPONENT
uvms.Jma = [zeros(1,7) v_kw' zeros(1,3)];

%VEHICLE ALTITUDE
uvms.Ja = [zeros(1,7) v_kw' zeros(1,3)];

%VECHILCE UNDERACTUATION
uvms.Jund = [zeros(6,7) eye(6)];

uvms.Jstop = [zeros(6,7),eye(6)];
end