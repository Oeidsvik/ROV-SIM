function [R_el] = RotMatrix(deltaX,deltaY,deltaZ)
%% Calculate rotation angles (relative to NED)
Ri=[0,0,1;0,1,0;-1,0,0];
R_el=zeros(6,6);
if deltaY==0
    phi=0;
else
    phi= asind(deltaY/(deltaZ));
end
if deltaZ==0
    theta=0;
else
    theta= asind(deltaX/(deltaZ));
end
%% Calculate rotation matrix
Ry=[sind(theta),0,cosd(theta);0,1,0;-cosd(theta),0,-sind(theta)];
Rx=[1,0,0;0,1,0;0,0,1];
Rz=[cosd(phi),sind(phi),0;sind(phi),cosd(phi),0;0,0,1];
Rz=(Ri*Rz)*Ri';
R=Rz*Ry*Rx;
R_el(1:3,1:3)=R;
R_el(4:6,4:6)=R;
end

