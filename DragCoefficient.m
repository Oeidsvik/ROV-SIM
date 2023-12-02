function [Cd] = DragCoefficient(D)
%% find drag and mass coefficients based on Reynolds number
% independent of KC, as KC is large for all practical purposes 
% CD(KC)= CD(steady) for KC>30-50
Cd=[50,1.6;100,1.4;200,1.3;500,1.2;1000,1;2000,0.95;5000,1;10^4,1.05;2*10^4,1.1;...
       5*10^4,1.15;10^5,1.20;2*10^5,1.20;5*10^5,0.8;10^6,0.4;10^7,0.8];
   
Cd(:,1)=Cd(:,1)*(10^-6)/D;   
   
end

