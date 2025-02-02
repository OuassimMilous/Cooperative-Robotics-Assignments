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
uvms.wTr = eye(4,4);
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

uvms.a = [];


% horizental
uvms.A.ha = 0;
uvms.xdot.ha = 0;
uvms.Jha = zeros(1,13);  

% minimum altitude
uvms.A.ma= 0;
uvms.xdot.ma = 0;
uvms.Jma = zeros(1,13);

% vehicule control
uvms.A.v = zeros(6);
uvms.xdot.v = zeros(6,1);
uvms.Jv = zeros(6,13); 

% get closer
uvms.A.closer = zeros(3);
uvms.xdot.closer = zeros(3,1);
uvms.Jcloser = zeros(3,13); 

% landing 
uvms.A.landing= 0;
uvms.xdot.landing = 0;
uvms.Jlanding = zeros(1,13);

% tool
uvms.A.tool = zeros(6);
uvms.xdot.tool = zeros(6,1);
uvms.Jtool = zeros(6,13);

end

