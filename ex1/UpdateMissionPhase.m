function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  %safe navigation
            [ang_e, lin_e] = CartError(uvms.wTgv , uvms.wTv);
            if ( norm(ang_e)  < 0.05 &&  norm(lin_e) <0.1 ) 
                 mission.phase = 2;
                 mission.phase_time = 0;
                disp(' *** change to phase 2');
            end 

        case 2 % ROCK ALIGNMENT
            % TODO: the condition to go to phase 3
            [ang_rock, lin_rock] = CartError(uvms.wTr , uvms.wTv);
            if ( norm(ang_rock)  < 0.01 )
                mission.phase = 3;
                mission.phase_time = 0;
                disp(' *** change to phase 3');
            end
        case 3 % safe landing Task
            
            
    end
end

