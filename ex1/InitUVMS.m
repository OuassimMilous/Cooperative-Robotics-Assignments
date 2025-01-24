function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% to be computed at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.sensorDistance = 0;

uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jha = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];
%DEFINE THE JACOBIAN VARIABLES
uvms.Jv_l = []; 
uvms.Jv_a = [];
%HORIZONTAL TASK JACOBIAN
uvms.Jha = [];  
%MINIMUM ALTITUDE TASK
uvms.Jma = [];
%ALTITUDE TASK
uvms.Ja = [];
%FOR UNDERACTUATION TASK
uvms.Jund = [] ;



uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.ha = [];
uvms.xdot.t = [];
% DEFINE THE VARIABLES
uvms.xdot.v_l = [];
uvms.xdot.v_a = [];
% FOR MAINTAINING THE HORIZONTAL DISTANCE
uvms.xdot.ha = [];
%MINIMUM ALTITUDE TASK
uvms.xdot.ma = [];
%ALTITUDE TASK
uvms.xdot.a = [];
% FOR UNDERACTUATION
uvms.xdot.und = [];



    
uvms.A.jl = zeros(7,7);
uvms.A.mu = 0;
uvms.A.t = zeros(6,6);
%DEFINE THE ACTIVATION FUNCTION
uvms.A.v_l = zeros(3);
uvms.A.v_a = zeros(3);
%HORIZONATAL TASK ACTIVATION
uvms.A.ha = 0;
%MINIMUM ALTITUDE TASK
uvms.A.ma= 0;
%ALTITUDE TASK
uvms.A.a= 0;
%FOR UNDERACTUATION
uvms.A.und = zeros(6);



%DEFINE THE ACTIVATION FUNCTION - FOR TASK SWITCHING
uvms.Ap.v_l = zeros(3);
uvms.Ap.v_a = zeros(3);
%HORIZONATAL TASK ACTIVATION - FOR TASK SWITCHING
uvms.Ap.ha = 0;
%MINIMUM ALTITUDE TASK - FOR TASK SWITCHING
uvms.Ap.ma= 0;
%ALTITUDE TASK - FOR TASK SWITCHING
uvms.Ap.a= 0;




uvms.v_rho_ha = [];
uvms.v_n_ha = [];
uvms.a = [];



end

