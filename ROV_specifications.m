function [D_rov_quad,D_rov_lin,M_rov,M_rov_a,BG_rov,W_rov,B_rov,UG_rov,C_gains,T_limits] = ROV_specifications(g,ROV,Controller_type)
%% ROV Parameter Matrices ( low speed, 3 planes of symmetry)
if strcmp(ROV,'Sf30k')
% Mass
M_rov=[1826,0,0,0,0,0;          
       0,1826,0,0,0,0;
       0,0,1826,0,0,0;
       0,0,0,525,0,0;
       0,0,0,0,794,0;
       0,0,0,0,0,691];  
   % Added Mass
M_rov_a =[629 ,0,0,0,0,0;       
          0,1300,0,0,0,0;
          0,0,4113,0,0,0;
          0,0,0,550,0,0;
          0,0,0,0, 962,0
          0,0,0,0,0, 200];
      % Quadratic damping
D_rov_quad=[472,0,0,0, 0,0;    
            0,1109,0,0,0,0;
            0,0,1500,0,0,0;
            0,0,0,200,0,0
            0,0,0,0, 625,0;
            0,0,0,0,0, 200];
        % linear damping
D_rov_lin=[76,0,0,0,0,0;    
           0,200,0,0,0,0;
           0,0,240,0,0,0;
           0,0,0,200,0,0;
           0,0,0,0,171,0;
           0,0,0,0,0,200];            
 
% ROV Restoring parameters ( BG= G(x,y,z)-B(x,y,z), vertical is positive downwards)
BG_rov=[0;0;0.4];
W_rov=M_rov(1,1)*g;
B_rov=W_rov;
% Position of umbilical cable relative to COG[m].( UG= G(x,y,z)-U(x,y,z)  )
UG_rov=[0;0;0.4];      

if strcmp(Controller_type,'PID') 
% PID controller gains (surge:Kp,Ki,Kd; depth:Kp,Ki,Kd; yaw: Kp,Ki,Kd)
C_gains =[3000,200,1500;4000,500,1500;10000,1000,4000];
elseif strcmp(Controller_type,'MB')
% Modelbased controller gains (surge:Kp,Ki,Kd; depth:Kp,Ki,Kd; yaw: Kp,Ki,Kd)
C_gains =[3000,200,1500;4000,500,1500;10000,1000,4000];
end
% Thruster Limits (surge, sway, heave, yaw)
T_limits=[1200,1000,1400,400];


elseif strcomp(ROV,'BlueROV')
    % Mass
M_rov=[10,0,0,0,0,0;      
       0,10,0,0,0,0;
       0,0,10,0,0,0;
       0,0,0,0.1,0,0;
       0,0,0,0,0.1,0;
       0,0,0,0,0,0.1]; 
   % Added Mass
M_rov_a =[7.04 ,0,0,0,0,0;      
          0,18.5,0,0,0,0;
          0,0,13.3,0,0,0;
          0,0,0,0.2,0,0;
          0,0,0,0, 0.2,0
          0,0,0,0,0, 0.2];      
      % Quadratic damping
D_rov_quad=[16.5,0,0,0, 0,0;     
            0,46.5,0,0,0,0;
            0,0,37.63,0,0,0;
            0,0,0,0.5,0,0
            0,0,0,0, 0.5,0;
            0,0,0,0,0, 1];      
          % linear damping   
 D_rov_lin=[2,0,0,0,0,0;   
           0,4,0,0,0,0;
           0,0,5,0,0,0;
           0,0,0,1,0,0;
           0,0,0,0,1,0;
           0,0,0,0,0,1]; 
% ROV Restoring parameters ( BG= G(x,y,z)-B(x,y,z), vertical is positive downwards)
BG_rov=[0;0;0.1];
W_rov=M_rov(1,1)*g;
B_rov=W_rov;
% Position of umbilical cable relative to COG[m].( UG= G(x,y,z)-U(x,y,z)  )
UG_rov=[-0.2;0;0.0];    

% controller gains (surge:Kp,Ki,Kd; vertical:Kp,Ki,Kd; yaw: Kp,Ki,Kd)
C_gains =[300,20,150;300,50,150;300,10,100];
else
disp('undefied ROV')
return
end


end

