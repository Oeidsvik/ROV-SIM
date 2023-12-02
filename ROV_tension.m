function [T_global,T_rov] = ROV_tension (R_rov,UG_rov,N,Restoring)
% Global Tension forces acting on ROV
T_global=-[(Restoring(N*6-2)),Restoring(N*6-1),(Restoring(N*6))]';

% Tension forces given in ROV-frame
T_rov=R_rov(1:3,1:3)'*T_global;
 
% Tension moments ( r x T)
 M_rov=[UG_rov(2)*T_rov(3)-UG_rov(3)*T_rov(2),...
                    UG_rov(3)*T_rov(1)-UG_rov(1)*T_rov(3),...
                    UG_rov(1)*T_rov(2)-UG_rov(2)*T_rov(1)];
% Tension forces and moments 
 T_rov(1:6)=[T_rov,M_rov'];
end

