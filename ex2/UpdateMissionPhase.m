function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                 [ang_l, lin_l] = CartError(pandaArm.ArmL.wTg,pandaArm.ArmL.wTt);
                 [ang_r, lin_r] = CartError(pandaArm.ArmR.wTg,pandaArm.ArmR.wTt);
                 if(norm(ang_l)<0.0175 && norm(lin_l)<0.001 ...
                         && norm(ang_r)<0.0175 && norm(lin_r)<0.001)
                     mission.phase = 2;
                     mission.phase_time = 0;
                     disp('Changing to phase 2');
                 end
                 
                % max error: 1/10 cm and 1deg
                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task

                 [ang, lin] = CartError(pandaArm.wTog,pandaArm.ArmL.wTt);
                 if(norm(ang)<0.0524 && norm(lin)<0.001 ...
                         && norm(ang)<0.0524 && norm(lin)<0.001)
                     mission.phase = 3;
                     mission.phase_time = 0;
                     disp('Changing to phase 3');
                     
                 end
                    
                % max error: 1 cm and 3deg
               
            case 3 % Finish motion
                if(mission.phase_time >= 5)
                    mission.phase = 4;
                     mission.phase_time = 0;
                  disp('Changing to phase 4');
                end
            % case 4 end
             end
end

