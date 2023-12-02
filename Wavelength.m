function [k] = Wavelength(g,T,h)
% This function estimates the wave number(2*pi/wavelength) of the sea state
% using Newton Rhapson iteration

%% Variables
syms k;
if T==0
    k=0;
else
    omega=(2*pi/T);                 % Frequency for deep water wave
    k0=omega*omega/g;               % Deep water wave number (initial solution)
    e = 1e-2;                       % Setting the maximum error value
    dk = e + 1;                     % Error interval
    
    %% defining function f AKA. Dispersion relation
    f=k*tanh(k*h)-omega*omega/g;
    
    %% initial Values
    k = k0;                         % initially assumed value of "k"
    count = 0;                      % setting counter to know the number of iterations
    p = zeros(1,1);
    
    %% Iteration( Newton-Rhapson )
    while (abs(dk) > e)             % initialising the iteration and continue until the error is less than tolerance
        dk = eval(f/(diff(f)));     % calculating dk, diff is used for finding the differentiation of the fuction
        k = k - dk;                 % updating the value of l
        count = count + 1;          % incrimenting the counter
        p(count) = k;               % Iteration Values
        if (count > 200)
            fprintf('Error...! Solution not converging !!! \n');  % printing the error message
            break;
        end
        
    end
end
end
