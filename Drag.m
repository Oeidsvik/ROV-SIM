function [P_g,Cable_rov_drag] = Drag(Vr,N,rho,D,E_length,Cd,Ct,R_g)
F_d=zeros(N+1,3);
P_g=zeros(N*6,1);
Cd_Vr=zeros(length(Vr),2);

%% Drag coefficents for different velocities (Reynolds dependent Cd)
for i=1:N
    [~,index]=min(abs(Cd(:,1)-abs(sqrt(Vr(i,2)^2+Vr(i,3)^2))));
    Cd_Vr(i,2)=Cd(index,2);
end

%% Cable drag forces
for i=1:N
    %% According to cross flow princ. long. and late. are independent
    Vr_h= sqrt(Vr(i,2)^2+Vr(i,3)^2);
    %% angle of horisontal drag force
    if Vr(i,3)==0
        alpha=90;
    else
        alpha=atand(abs(Vr(i,2))/abs(Vr(i,3)));
    end
    
    %% Drag Forces on each cable element
    F_d(i,3)=- sign(Vr(i,3))*cosd(alpha)*0.5*rho*Cd_Vr(i,2)*E_length(i,1)*D*Vr_h*Vr_h;
    F_d(i,2)=- sign(Vr(i,2))*sind(alpha)*0.5*rho*Cd_Vr(i,2)*E_length(i,1)*D*Vr_h*Vr_h;
    F_d(i,1)=- 0.5*rho*Ct*E_length(i,1)*pi*D*Vr(i,1)*abs(Vr(i,1));
end
%% convert from element forces to global nodal forces
% 1st Node (half element 1 drag force).
P_g(1:3,1)=R_g(1:3,1:3)*0.5*F_d(1,:)';
% 2nd Node (half element 1 and element 2 drag forces.
P_g(6-2:6,1)=R_g(1:3,1:3)*0.5*F_d(1,:)';
P_g(6-2:6,1)=P_g(6-2:6,1)+R_g(4:6,4:6)*0.5*F_d(2,:)';
% intermediate elements
for i=2:N-1
    P_g(i*6-2:i*6,1)=R_g(i*6-2-6:i*6-6,i*6-2-6:i*6-6)*0.5*F_d(i,:)';
    P_g(i*6-2:i*6,1)=P_g(i*6-2:i*6,1)+R_g(i*6-2:i*6,i*6-2:i*6)*0.5*F_d(i+1,:)';
end

%% ROV-cable drag force (half of the element drag of last element),global reference frame
Cable_rov_drag=R_g(N*6-2:N*6,N*6-2:N*6)*0.5*F_d(N,:)';

end



