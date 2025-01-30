function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1
            uvms.Ap.v = 1; % velocity control
            uvms.Ap.ha = 1; % horizental 
            uvms.Ap.ma= 1; % minimum altitude
            uvms.Ap.landing= 0; % altitude control
            uvms.Ap.tool = 0; % tool control task
            uvms.Ap.rock = 0; %  THE ROCK TASK
            uvms.Ap.closer = 0; % closer
    
         case 2
             uvms.Ap.v = 0;
             uvms.Ap.ha = 1; 
             uvms.Ap.ma= DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); %1; 
             uvms.Ap.landing= 0; %IT IS A LANDING TASK
             uvms.Ap.tool = 0; % eye(6) * IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
             uvms.Ap.rock = 1; %IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); %ACTIVATING THE ROCK TASK
             uvms.Ap.closer = 0; 

        case 3
             uvms.Ap.v = 0;
             uvms.Ap.ha = 0;
             uvms.Ap.ma= 0; %DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); 
             uvms.Ap.landing= 1; %IT IS A LANDING TASK
             uvms.Ap.tool = 0; %eye(6) * IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
             uvms.Ap.rock = 0; %ACTIVATING THE ROCK TASK
             [ang_closer, lin_closer] = CartError(uvms.wTg , uvms.wTb);
             uvms.Ap.closer = 1 * IncreasingBellShapedFunction(0,1.2,0,1,norm(lin_closer(1:2))); %closer

        case 4
             uvms.Ap.v = 0;
             uvms.Ap.ha = 0;
             uvms.Ap.ma= 0; %DecreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); 
             uvms.Ap.landing= 0; %IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time); %IT IS A LANDING TASK
             uvms.Ap.tool = IncreasingBellShapedFunction(0, 1, 0, 1, mission.phase_time);
             uvms.Ap.rock = 0; %ACTIVATING THE ROCK TASK
             uvms.A.stop = eye(6);
             uvms.Ap.closer = 0; 
   end 
    
% arm tool position control
uvms.A.tool = eye(6) * uvms.Ap.tool;

%ACTIVATION FUNCTION FOR ROCK
uvms.A.rock = eye(3) * uvms.Ap.rock;

%ACTIVATION FUNCTION FOR POSITION AND ORIENTATION CONTROL TASK FOR VEHICLE. 
uvms.A.v = eye(6) * uvms.Ap.v;


%HORIZONTAL ACTIVATION FUNCTION DEFINATION
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho_ha)) * uvms.Ap.ha;
% delta is 0.2 and delta - 1 is 0.1 and output should in between 0 and 1.
%norm(uvms.v_rho_ha) is theta for make the vehicle horizontal

%MINIMUM ALTITUDE
uvms.A.ma = DecreasingBellShapedFunction(0.5, 1 , 0, 1, uvms.a) * uvms.Ap.ma;
%We activate this MINIMUM ALTITUDE function when my vehicle goes below 2m and we dont care
%about when it goes above 3

%ALTITUDE ACTIVATION FUNCTION
uvms.A.landing = 1 * uvms.Ap.landing;


%closer
% uvms.A.closer = IncreasingBellShapedFunction(0, 1, 0, 1, norm(uvms.) * uvms.Ap.closer;
uvms.A.closer = eye(3)*uvms.Ap.closer;


%VECHILCE UNDERACTUATION
uvms.A.und = diag([0 0 0 1 0 0]); %OMEGA X IS UNDERACUTATED     