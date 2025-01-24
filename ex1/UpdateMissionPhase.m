function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  
            % add policy for changing phase
            %WE ARE DOING ACTION CHANGE IN BETWEEN GOING TO POSITION THEN WHEN IT REACHES
            %OVER THER THEN LAND ON THAT POSITION
            [ang_e, lin_e] = CartError(uvms.wTgv , uvms.wTv);
            if ( norm(ang_e)  < 0.05 &&  norm(lin_e) <0.1 ) 
                mission.phase = 2;
                mission.phase_time = 0;
                disp(' *** change to phase 2');
            end
       
            %THIS IS NAIVE BECAUSE HERE WE ASSUMED THAT CASE 2 WILL ACTIVATE ONCE THE
            %GO-TO FUNCTION(CASE 1) FINISHED. LIKE IT COULD BE COME FROM ANY CASE. 

        case 2
        % NOW WE DONT HAVE ANYTHING TO DO, IN CASE 2

    end
end

