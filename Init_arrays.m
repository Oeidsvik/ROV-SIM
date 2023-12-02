function [a_rov,v_rov,u_rov,disp_count,...
     Dispvector,u,E_control] = Init_arrays(N,T,u_0)
%% Defining empty initial matrices
% umbilical----------------------------------------------------------------
u=u_0;
% initial ROV state
a_rov=zeros(6,1);
v_rov=zeros(6,1);
u_rov=zeros(6,1);
u_rov(1:3)=u_0(N*6-2:N*6);

disp_count=1;
%integral error terms(for PID)
E_control=zeros(6,1);
% Display vector( Display simulation information every 10 seconds)
counter=1;
for i=10:10:T+10
    Dispvector(counter)=i;
    counter=counter+1;
end

end

