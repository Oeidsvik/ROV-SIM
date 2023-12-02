function [Tau_NED,Tau_body,E_control] = PID(dt,t,u_rov,R_rov,v_rov,rov_ref,E_control,C_gains,T_limits)

%%                           Description:
%**************************************************************************
%    This controller employs a simple PID control:
%    for the surge speed of the ROV
%    for the depth of the ROV
%    for the heading of the ROV
%**************************************************************************

%% Speed    ---------------------------------------------------------------
%Proportional
P_speed=R_rov(1:3,1:3)'*v_rov(1:3)-[rov_ref(round(t/dt)+1,2);0;0];
% Integral
I_speed=E_control(2)+P_speed(1)*dt;
% Derivative
D_speed=(-E_control(1)+P_speed(1))/dt;
% update Error variable
E_control(1)=P_speed(1);
E_control(2)=I_speed;
% calculate surge thrust
Tau_body(1,1)=(-C_gains(1,1)*P_speed(1) -C_gains(1,2)*I_speed - C_gains(1,3)*D_speed);

%% Heading  ---------------------------------------------------------------
%Proportional
P_yaw=u_rov(4:6)-[0;0;rov_ref(round(t/dt)+1,3)];
% Integral
I_yaw=E_control(4)+P_yaw(3)*dt;
% Derivative
D_yaw=(-E_control(3)+P_yaw(3))/dt;
% update Error variable
E_control(3)=P_yaw(3);
E_control(4)=I_yaw;
% calculate yaw thrust
Yaw_local=R_rov(4:6,4:6)'*[0;0;-C_gains(3,1)*P_yaw(3)-C_gains(3,2)*I_yaw-C_gains(3,3)*D_yaw];
Tau_body(6,1)=Yaw_local(3);

%% depth    ---------------------------------------------------------------
% Proportional
P_depth=u_rov(1:3)-[0;0;rov_ref(round(t/dt)+1,4)];
% Integral
I_depth=E_control(6)+P_depth(3)*dt;
% Derivative
D_depth=(-E_control(5)+P_depth(3))/dt;
% update Error variable
E_control(5)=P_depth(3);
E_control(6)=I_depth;
% calculate heave thrust
Heave_local=R_rov(1:3,1:3)'*[0;0;-C_gains(2,1)*P_depth(3)-C_gains(2,2)*I_depth-C_gains(2,3)*D_depth];
Tau_body(3,1)=Heave_local(3);

%% Check thruster limits
for i=1:3
    if Tau_body(i)>T_limits(i)
        Tau_body(i)=T_limits(i);
    elseif Tau_body(i)<-T_limits(i)
        Tau_body(i)=-T_limits(i);
    end
end
if Tau_body(6)>T_limits(4)
    Tau_body(6)=T_limits(4);
elseif Tau_body(6)<-T_limits(4)
    Tau_body(6)=-T_limits(4);
end

% Global thruster forces and moments
Tau_NED=R_rov*Tau_body;

end

