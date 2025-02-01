function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1
            uvms.Ap.ha = 1; % horizental 
            uvms.Ap.ma= 1; % minimum altitude
            uvms.Ap.v = 1; % vehicule control
            uvms.Ap.landing= 0; % altitude control
            uvms.Ap.tool = 0; % tool control task
    
         case 2
             uvms.Ap.ha = 1; 
             uvms.Ap.ma= 1;
             uvms.Ap.v = 1;
             uvms.Ap.landing= 0;
             uvms.Ap.tool = 0;

        case 3
             uvms.Ap.ha = 0;
             uvms.Ap.ma= 0; 
             uvms.Ap.v = IncreasingBellShapedFunction(0,0.6,0,1,norm(uvms.err.lin_closer));
             uvms.Ap.landing= 1;
             uvms.Ap.tool = 0;
             
        case 4
             uvms.Ap.ha = 1;
             uvms.Ap.ma= 0; 
             uvms.Ap.v = 1;
             uvms.Ap.landing= 0; 
             uvms.Ap.tool = 1;
   end 
    
%HORIZONTAL ACTIVATION FUNCTION DEFINATION
uvms.A.ha =  IncreasingBellShapedFunction(0.1, 0.2, 0, 1, norm(uvms.v_rho_ha)) * uvms.Ap.ha;

%MINIMUM ALTITUDE
uvms.A.ma = DecreasingBellShapedFunction(1, 2 , 0, 1, uvms.a) * uvms.Ap.ma;

% vehicule control
uvms.A.v = eye(6) * uvms.Ap.v;

%ALTITUDE ACTIVATION FUNCTION
uvms.A.landing = 1 * uvms.Ap.landing;

% arm tool position control
uvms.A.tool = eye(6) * uvms.Ap.tool;

end