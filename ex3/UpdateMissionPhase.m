function [pandaArm, pandaArm2, mission] = UpdateMissionPhase(pandaArm, pandaArm2, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                 if(norm(pandaArm.ang_l)<0.0175 && norm(pandaArm.lin_l)<0.001 ...
                         && norm(pandaArm2.ang_r)<0.0175 && norm(pandaArm2.lin_r)<0.001)
                     mission.phase = 2;
                     mission.phase_time = 0;
                     disp('Changing to phase 2');
                      pandaArm.tTo = inv(pandaArm.wTt)*pandaArm.wTo;
                      pandaArm2.tTo = inv(pandaArm2.wTt)*pandaArm2.wTo;

                 end
                % max error: 1/10 cm and 1deg
                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task

                 if(norm(pandaArm.ang_l)<0.0524 && norm(pandaArm.lin_l)<0.001 ...
                         && norm(pandaArm2.ang_r)<0.0524 && norm(pandaArm2.lin_r)<0.001)
                     mission.phase = 3;
                     mission.phase_time = 0;
                     disp('Changing to phase 3');
                     
                 end
                % max error: 1 cm and 3deg
               
            case 3 % Finish motion
                if(mission.phase_time >= 5)
                    mission.phase = 4;
                    mission.phase_time = 0;
                  disp('finishing');
                end
            case 4 % end
        end
                
end

