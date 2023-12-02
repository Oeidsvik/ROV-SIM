function [Tau_NED,Tau_body,E_control] = MB_controller(dt,t,u_rov,a_rov,R_rov,v_rov,rov_ref,E_control,C_gains,T_limits,...
                                                      D_rov_lin,D_rov_quad,M_rov,M_rov_a,g_eta,L)
% ROV speed in local frame
v_rov_local=R_rov'*v_rov;
%% Desired states 
if t==0
 % speed
v_d(1,1)=rov_ref(round(t/dt)+1,2);                                          % surge
v_d(6,1)=(rov_ref(round(t/dt)+1,3))/dt;                                     % heading
v_d(3,1)=(rov_ref(round(t/dt)+1,4)+L)/dt;                                   % depth

% acceleration (local)
a_d(1,1)=(rov_ref(round(t/dt)+1,2))/dt;                                     % surge
a_d(6,1)=(rov_ref(round(t/dt)+1,3))/dt^2 ;                                  % heading
a_d(3,1)=(rov_ref(round(t/dt)+1,4)+L)/dt^2;                                  % depth   
elseif t==dt
 % speed
v_d(1,1)=rov_ref(round(t/dt)+1,2);                                          % surge
v_d(6,1)=(rov_ref(round(t/dt)+1,3)-rov_ref(round(t/dt),3))/dt;           	% heading
v_d(3,1)=(rov_ref(round(t/dt)+1,4)-rov_ref(round(t/dt),4))/dt;            	% depth

% acceleration (local)
a_d(1,1)=(rov_ref(round(t/dt)+1,2)-rov_ref(round(t/dt),2))/dt;              % surge
a_d(6,1)=(-(rov_ref(round(t/dt),3))+rov_ref(round(t/dt)+1,3))/dt^2;         % heading
a_d(3,1)=(-(rov_ref(round(t/dt),4))+rov_ref(round(t/dt)+1,4))/dt^2;         % depth   
else
 % speed
v_d(1,1)=rov_ref(round(t/dt)+1,2);                                          % surge
v_d_int=R_rov(4:6,4:6)'*[0;0;(rov_ref(round(t/dt)+1,3)-rov_ref(round(t/dt),3))/dt] ;	
v_d(6,1)=v_d_int(3);                                                        % heading
v_d_int=R_rov(1:3,1:3)'*[0;0;(rov_ref(round(t/dt)+1,4)-rov_ref(round(t/dt),4))/dt];	
v_d(3,1)=v_d_int(3);                                                        % depth
% acceleration (local)
a_d(1,1)=(rov_ref(round(t/dt)+1,2)-rov_ref(round(t/dt),2))/dt;              % surge
a_d_int=R_rov(4:6,4:6)'*[0;0;(-2*(rov_ref(round(t/dt),3))+rov_ref(round(t/dt)-1,3)+rov_ref(round(t/dt)+1,3))/dt^2];
a_d(6,1)=a_d_int(3) ;                                                       % heading
a_d_int=R_rov(1:3,1:3)'*[0;0;(-2*(rov_ref(round(t/dt),4))+rov_ref(round(t/dt)-1,4)+rov_ref(round(t/dt)+1,4))/dt^2];
a_d(3,1)=a_d_int(3);                                                        % depth   
end


%% Error terms
%Speed
P_speed=v_rov_local(1:3)-[rov_ref(round(t/dt)+1,2);0;0];
D_speed=(P_speed(1)-E_control(1))/dt;
E_control(1)=P_speed(1);
E_control(2)=0;
% Heading
P_yaw=u_rov(4:6)-[0;0;rov_ref(round(t/dt)+1,3)];
D_yaw=(-E_control(3)+P_yaw(3))/dt;
E_control(3)=P_yaw(3);
E_control(4)=0;
% Depth
P_depth=u_rov(1:3)-[0;0;rov_ref(round(t/dt)+1,4)];
D_depth=(-E_control(5)+P_depth(3))/dt;
E_control(5)=P_depth(3);
E_control(6)=0;
%C_gains=C_gains*0.1;
%% Thruster Forces
%Surge
Tau_body(1,1)=((M_rov(1,1)+M_rov_a(1,1))*a_d(1,1)+D_rov_quad(1,1)*v_d(1,1)*abs(v_d(1,1))+...
                D_rov_lin(1,1)*v_d(1,1)+g_eta(1,1));

PD=-C_gains(1,1)*P_speed(1) - C_gains(1,3)*D_speed;
Tau_body(1,1)=Tau_body(1,1)+PD;
%Heading
Tau_body(6,1)=((M_rov(6,6)+M_rov_a(6,6))*a_d(6,1)+D_rov_quad(6,6)*v_d(6,1)*abs(v_d(6,1))+...
                D_rov_lin(6,6)*v_d(6,1)+g_eta(6,1))       ;
PD=R_rov(4:6,4:6)'*[0;0;-C_gains(3,1)*P_yaw(3)-C_gains(3,3)*D_yaw];           
Tau_body(6,1)=Tau_body(6,1)+PD(3); 
 
%Depth
Tau_body(3,1)=((M_rov(3,3)+M_rov_a(3,3))*a_d(3,1)+D_rov_quad(3,3)*v_d(3,1)*abs(v_d(3,1))+...
                D_rov_lin(3,3)*v_d(3,1)+g_eta(3,1))        ;              
PD =R_rov(1:3,1:3)'*[0;0;-C_gains(2,1)*P_depth(3)-C_gains(2,3)*D_depth];
Tau_body(3,1)=Tau_body(3,1)+PD(3);            
 
Tau_NED=R_rov*Tau_body;

end

