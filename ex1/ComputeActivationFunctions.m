function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here
    switch mission.phase
        case 1
            prev = mission.actions.go_to.tasks;
            current = mission.actions.go_to.tasks;
            
         case 2
            prev = mission.actions.go_to.tasks;
            current = mission.actions.align.tasks;
            
        case 3
            prev = mission.actions.align.tasks;
            current = mission.actions.land.tasks;
             
        case 4
            prev = mission.actions.land.tasks;
            current = mission.actions.manip.tasks;
   end 
    
%HORIZONTAL ACTIVATION FUNCTION DEFINATION
uvms.A.ha =  IncreasingBellShapedFunction(0.01, 0.02, 0, 1, norm(uvms.v_rho_ha)) * ActionTransition("HA", prev, current, mission.phase_time);

%MINIMUM ALTITUDE
uvms.A.ma = DecreasingBellShapedFunction(1, 2 , 0, 1, uvms.a) * ActionTransition("MA", prev, current, mission.phase_time);

% vehicule control
uvms.A.v = eye(6) * ActionTransition("V", prev, current, mission.phase_time);

%ALTITUDE ACTIVATION FUNCTION
uvms.A.landing = 1 * ActionTransition("L", prev, current, mission.phase_time);

% arm tool position control
uvms.A.tool = eye(6) * ActionTransition("T", prev, current, mission.phase_time);

% get closer
uvms.A.closer = eye(3) * IncreasingBellShapedFunction(0,0.6,0,1,norm(uvms.err.lin_closer)) *  ActionTransition("C", prev, current, mission.phase_time);

end