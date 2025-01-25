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
            [ang_rock, lin_rock] = CartError(uvms.wTr , uvms.wTv);
            if ( norm(ang_rock)  < 0.05 )
                mission.phase = 3;
                mission.phase_time = 0;
                disp(' *** change to phase 3');
            end
<<<<<<< Updated upstream
        case 3 % safe landing 
           
            if ( norm((0 - uvms.a))  < 0.05 )
=======
        case 3 % safe landing Task
            [ang_rock, lin_rock] = CartError(uvms.wTr , uvms.wTv);
            if ( norm(ang_rock)  < 0.01 )
>>>>>>> Stashed changes
                mission.phase = 4;
                mission.phase_time = 0;
                disp(' *** change to phase 4');
            end
<<<<<<< Updated upstream
        case 4 % tool 

=======
        case 4  
>>>>>>> Stashed changes
            
    end
end

