function [X] = Error_check(a_new,u_rov)

%% This function checks whether the next simulated value is real
    Real=isreal(a_new);
    X='false';
    if Real==1
    else
        disp('Response grows exponentially, consider reducing step size')
        X='true';
        return
    end
    Nan=isnan(a_new(4,1));
    if Nan==1
        disp('Response grows exponentially, consider reducing step size')
        X='true';
        return
    end

    if abs(u_rov(4,1))>=pi/2
        X='true';
        disp('Roll angle larger than 90 degrees')
        return
    elseif abs(u_rov(5,1))>pi/2
        disp(' Pitch angle larger than 90 degrees')
         X='true';
         return
    end
    
    
end

