function [R_rov,g_eta] = Transformation_rov(u_rov,B_rov,W_rov,BG_rov)

%% obtain euler angles from ROV rotation ( 10 decimals accuracy)
phi=round(u_rov(4,1)*180/pi,10);
theta=round(u_rov(5,1)*180/pi,10);
psi=round(u_rov(6,1)*180/pi,10);


%% Translational Transformation for ROV between Body and NED
R_rov=[cosd(psi)*cosd(theta),-sind(psi)*cosd(phi)+cosd(psi)*sind(theta)*sind(phi)...
         ,sind(psi)*sind(phi)+cosd(psi)*cosd(phi)*sind(theta);
         sind(psi)*cosd(theta),cosd(psi)*cosd(phi)+sind(phi)*sind(theta)*sind(psi),...
         -cosd(psi)*sind(phi)+sind(theta)*sind(psi)*cosd(phi);...
         -sind(theta),cosd(theta)*sind(phi),cosd(theta)*cosd(phi)];

%% Angular transformation for ROV between Body and NED
 T_rov=[1,sind(phi)*tand(theta),cosd(phi)*tand(theta);...
           0,cosd(phi),-sind(phi);...
           0,sind(phi)/cosd(theta),cosd(phi)/cosd(theta)]; 
       
%% Total ROV transformation matrix       
R_rov=[R_rov,zeros(3,3);zeros(3,3),T_rov];


%% ROV hydrostatic restoring
 g_eta=[(W_rov-B_rov)*sind(theta);...
     -(W_rov-B_rov)*cosd(theta)*sind(phi);...
     -(W_rov-B_rov)*cosd(theta)*cosd(phi);...
     -(BG_rov(2)*B_rov)*cosd(theta)*cosd(phi)+(BG_rov(3)*B_rov)*cosd(theta)*sind(phi);...
     +BG_rov(3)*B_rov*sind(theta)+BG_rov(1)*cosd(theta)*cosd(phi);
     -BG_rov(1)*B_rov*cosd(theta)*sind(phi)-BG_rov(2)*B_rov*sind(theta)];

 
end

