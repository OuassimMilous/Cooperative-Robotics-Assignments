function [pandaArm, mission] = UpdateMissionPhase(pandaArm, mission)    
        switch mission.phase
            case 1  %Go To Grasping Points
                % computing the errors for the go-to action defining tasks
                 if(norm(pandaArm.ArmL.ang)<0.0175 && norm(pandaArm.ArmL.lin)<0.001 ...
                         && norm(pandaArm.ArmR.ang)<0.0175 && norm(pandaArm.ArmR.lin)<0.001)
                     mission.phase = 2;
                     mission.phase_time = 0;
                     disp('Changing to phase 2');
                     pandaArm.ArmL.tTo = inv(pandaArm.ArmL.wTt)*pandaArm.ArmL.wTo;
                     pandaArm.ArmR.tTo = inv(pandaArm.ArmR.wTt)*pandaArm.ArmR.wTo;
                 end
                 
                % max error: 1/10 cm and 1deg
                
            case 2 % Cooperative Manipulation Start 
                % computing the errors for the rigid move-to task
                 if(norm(pandaArm.ArmL.ang)<0.0524 && norm(pandaArm.ArmL.lin)<0.001 ...
                         && norm(pandaArm.ArmR.ang)<0.0524 && norm(pandaArm.ArmR.lin)<0.001)
                     mission.phase = 3;
                     mission.phase_time = 0;
                     disp('Changing to phase 3');
                 end
                % max error: 1 cm and 3deg
            case 3 % Finish motion
                if(mission.phase_time >= 5)
                    mission.phase_time = 0;
                  disp('Changing to phase 4');
                end
        end
end

