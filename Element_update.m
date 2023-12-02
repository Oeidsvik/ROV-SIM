function [N,K_g_a,K_g,M_g_inv,M_g,Node_acc_new,Node_vel_new,Node_pos_new,Node_pos_init_new,Result,Delta_length,Surfaceship,Rayleigh]...
    = Element_update(A,E,I,l,N,Node_pos,Node_vel,Node_acc,K_g,K_g_a,K_e_a,...
    K_e,M,M_A,M_g,alpha,E_length_init,Node_pos_init,T2,Write_frequency,E_max,Result,Delta_length,Surfaceship,gamma,E_length)

% Update number of elements N
N=N+1;
%% in case E_max is different than 2, first element length must be updated
[K_e_elem_1,M_e_elem_1] = matrix_update(A,E,l,Delta_length,I,M,M_A,gamma,E_max,E_length_init);


%% update position, velocity and acceleration
% DeltaX=Node_pos(9) -Node_pos(1);
% DeltaY=Node_pos(10)-Node_pos(2);
% DeltaZ=Node_pos(11)-Node_pos(3);
%  dz=-(dummy(4))/sqrt((DeltaX/DeltaZ)^2+(DeltaY/DeltaZ)^2+1);
%  dy=dz*DeltaY/DeltaZ;
%  dx=dz*DeltaX/DeltaZ;
%  
% zero3=zeros(5,1);
% zero4=zeros((T2+1)*Write_frequency,5);
% Node_pos_new=vertcat(zero3,Node_pos);
% Node_vel_new=vertcat(zero3,Node_vel);
% Node_acc_new=vertcat(zero3,Node_acc);
% Node_pos_init_new=vertcat(zero3,Node_pos_init);
% Delta_length=vertcat(Delta_length,zero3);
% Result=horzcat(Result,zero4);
% Surfaceship=vertcat(Surfaceship,zero3);
% for i=1:3
%     Node_pos_new(i)=Node_pos(i);
%     Node_vel_new(i)=Node_vel(i);
%     Node_acc_new(i)=Node_acc(i);
% end
%  % position of "NEW" node--------------------------------------------------
% Node_pos_new(4)=Node_pos_new(1)+Node_pos_new(9)*dummy(4)/(dummy(4)+E_length(1,1));
% Node_pos_new(5)=Node_pos_new(2)+Node_pos_new(10)*dummy(4)/(dummy(4)+E_length(1,1));
% Node_pos_new(6)=Node_pos_new(3)+Node_pos_new(11)*dummy(4)/(dummy(4)+E_length(1,1));
% % velocity of "NEW" node--------------------------------------------------
% Node_vel_new(4)=Node_vel_new(1)+Node_vel_new(9)*dummy(4)/(dummy(4)+E_length(1,1));
% Node_vel_new(5)=Node_vel_new(2)+Node_vel_new(10)*dummy(4)/(dummy(4)+E_length(1,1));
% Node_vel_new(6)=Node_vel_new(3)+Node_vel_new(11)*dummy(4)/(dummy(4)+E_length(1,1));
% % % Acceleration of "NEW" node-----------------------------------------------  
% Node_acc_new(4)=Node_acc_new(1)+Node_acc_new(9)*dummy(4)/(dummy(4)+E_length(1,1));
% Node_acc_new(5)=Node_acc_new(2)+Node_acc_new(10)*dummy(4)/(dummy(4)+E_length(1,1));
% Node_acc_new(6)=Node_acc_new(3)+Node_acc_new(11)*dummy(4)/(dummy(4)+E_length(1,1));
% Node_pos_init_new(11)=-l;
% Node_pos_init_new(6)=-dummy(4);
% 
% 
% 
%  
% for i=1:2
%     Node_pos_new(i+6)=0;
%     Node_vel_new(i+6)=0;
%     Node_acc_new(i+6)=0;
% end


end

