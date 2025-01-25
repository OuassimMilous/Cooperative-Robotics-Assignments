function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1
            uvms.Ap.v_l = 1; % linear velocity control
            uvms.Ap.v_a = 1; % angular velocity control
            uvms.Ap.ha = 1; % horizental 
            uvms.Ap.ma= 1; % minimum altitude
            uvms.Ap.a= 0; % altitude control
            uvms.Ap.t = 0; % tool control task
            uvms.Ap.r = 0; % NOT ACTIVATING THE ROCK TASK
    
         case 2
             uvms.Ap.v_l = 0;
             uvms.Ap.v_a = 1;
             uvms.Ap.ha = 1;
             uvms.Ap.ma= DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); 
             uvms.Ap.a= IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); %IT IS A LANDING TASK
             uvms.Ap.t = 0; % eye(6) * IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
             uvms.Ap.r = 1; %ACTIVATING THE ROCK TASK
    end 
% arm tool position control
% always active
uvms.A.t = eye(6) * uvms.Ap.t;

%ACTIVATION FUNCTION FOR ROCK
uvms.A.r = eye(3) * uvms.Ap.r;

%ACTIVATION FUNCTION FOR POSITION AND ORIENTATION CONTROL TASK FOR VEHICLE. 
uvms.A.v_l = eye(3) * uvms.Ap.v_l;
uvms.A.v_a = eye(3) * uvms.Ap.v_a;


%HORIZONTAL ACTIVATION FUNCTION DEFINATION
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho_ha)) * uvms.Ap.ha;
% delta is 0.2 and delta - 1 is 0.1 and output should in between 0 and 1.
%norm(uvms.v_rho_ha) is theta for make the vehicle horizontal

%MINIMUM ALTITUDE
uvms.A.ma = DecreasingBellShapedFunction(0.5, 1 , 0, 1, uvms.a) * uvms.Ap.ma;
%We activate this MINIMUM ALTITUDE function when my vehicle goes below 2m and we dont care
%about when it goes above 3

%ALTITUDE ACTIVATION FUNCTION
uvms.A.a = 1 * uvms.Ap.a;



%VECHILCE UNDERACTUATION
uvms.A.und = diag([0 0 0 1 0 0]); %OMEGA X IS UNDERACUTATED     