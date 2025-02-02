function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  %safe navigation
            if ( norm(uvms.err.ang_v)  < 0.05 &&  norm(uvms.err.lin_v) <0.1 ) 
                 mission.phase = 2;
                 mission.phase_time = 0;
                disp(' *** change to phase 2');
            end 

        case 2 % ROCK ALIGNMENT

            if (abs(uvms.err.rock ) < 0.005 )
                mission.phase = 3;
                mission.phase_time = 0;
                disp(' *** change to phase 3');
            end
        case 3 % safe landing 
           
            if ( norm((0 - uvms.a))  < 0.05 )
                mission.phase = 4;
                mission.phase_time = 0;
                disp(' *** change to phase 4');
            end
        case 4 % tool 
            if ( norm(uvms.err.ang_t)  < 0.05 &&  norm(uvms.err.lin_t) <0.1 ) 
                mission.phase = 4;
                mission.phase_time = 0;
                disp(' *** ENDING');
            end
    end
end

