function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1
            uvms.Ap.v = 1; % velocity control
            uvms.Ap.ha = 1; % horizental 
            uvms.Ap.ma= 1; % minimum altitude
            uvms.Ap.landing= 0; % altitude control
            uvms.Ap.tool = 0; % tool control task
            uvms.Ap.closer = 0; % closer
    
         case 2
             uvms.Ap.v = 1;
             uvms.Ap.ha = 1; 
             uvms.Ap.ma= 1;
             uvms.Ap.landing= 0;
             uvms.Ap.tool = 0;

        case 3
             uvms.Ap.v = IncreasingBellShapedFunction(0,0.6,0,1,norm(uvms.err.lin_closer));
             uvms.Ap.ha = 0;
             uvms.Ap.ma= 0; 
             uvms.Ap.landing= 1;
             uvms.Ap.tool = 0;
             

        case 4
             uvms.Ap.v = 1;
             uvms.Ap.ha = 0;
             uvms.Ap.ma= 0; 
             uvms.Ap.landing= 0; 
             uvms.Ap.tool = 1;
             uvms.Ap.closer = 0; 
   end 
    
% arm tool position control
uvms.A.tool = eye(6) * uvms.Ap.tool;


%ACTIVATION FUNCTION FOR POSITION AND ORIENTATION CONTROL TASK FOR VEHICLE. 
uvms.A.v = eye(6) * uvms.Ap.v;


%HORIZONTAL ACTIVATION FUNCTION DEFINATION
uvms.A.ha = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho_ha)) * uvms.Ap.ha;
% delta is 0.2 and delta - 1 is 0.1 and output should in between 0 and 1.
%norm(uvms.v_rho_ha) is theta for make the vehicle horizontal

%MINIMUM ALTITUDE
uvms.A.ma = DecreasingBellShapedFunction(1, 2 , 0, 1, uvms.a) * uvms.Ap.ma;


%ALTITUDE ACTIVATION FUNCTION
uvms.A.landing = 1 * uvms.Ap.landing;
