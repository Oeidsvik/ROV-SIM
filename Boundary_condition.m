function [ P_g,u_0,Node_vel,...
           R_g_i,Node_acc,ROV_pos] =...
           Boundary_condition( P_g,u_0,...
           u_f,Node_vel,Node_acc,R_g_i,N,Init_ROV_yaw)

%% Apply boundary conditions. No rotation at ROV and surface ship
ROV_pos=zeros(6,1);
% Remove Rotation DOFs at SHIP-------------------------------------------
ship=4;
for i=1:3
P_g(ship,:)=[];
u_0(ship,:)=[];
u_f(ship,:)=[];
Node_vel(ship,:)=[];
Node_acc(ship,:)=[];
R_g_i(ship,:)=[];
R_g_i(:,ship)=[];
end
% Remove Rotation DOFs at ROV--------------------------------------------
ROV=N*6+1;
for i=1:3
P_g(ROV,:)=[];
u_0(ROV,:)=[];
Node_vel(ROV,:)=[];
Node_acc(ROV,:)=[];
R_g_i(ROV,:)=[];
R_g_i(:,ROV)=[];
end

%% Defining initial ROV state
ROV_pos(1,1)=0;
ROV_pos(2,1)=0;
ROV_pos(3,1)=0;
ROV_pos(4,1)=Init_ROV_yaw;

