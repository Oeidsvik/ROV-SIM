function [ Current_local,Current_global ] = Current(V_wind,V_tide,N,h_w,h,...
                                u,Current_dir,R_g)

%% Current Profile (DNV recommended standard)
V_current_wind=zeros(N+1,1);
V_current_tide=zeros(N+1,1);
for i=1:N-1
    if abs(u(6*i))>h_w
        V_current_wind(i+1,1)=0;
    else
        V_current_wind(i+1,1) = ...
            0.015*V_wind*((h_w-abs((u(6*i))))/h_w)^(1/7);
    end
    V_current_tide(i+1,1) =V_tide*((h-abs(u(6*i)))/h).^(1/7);
end
V_current = V_current_wind + V_current_tide;
% End Nodes
V_current(1)=V_tide+V_wind*0.015;
if abs(u(6*N))>h_w
    V_current(N+1)=V_tide*((h-abs(u(6*N)))/h).^(1/7);
else
    V_current(N+1)=V_tide*((h-abs(u(6*N)))/h).^(1/7)...
        + 0.015*V_wind*((h_w-abs(u(6*N)))/h_w)^(1/7);
end
%% simple linerar current profile 
%  for i=1:N-1    
%      V_current(i+1,1) =V_tide*(1-abs(Node_pos_global(5*i+1))/h);
%  end
%  
%  % End Node
%  V_current(1)=V_tide;
%  V_current(N+1,1)= V_tide*(1-abs(Node_pos_global(5*N+1))/h);


%% Global Current Vector
count=2;
Current_global=zeros(N*6,1);
for i=5:6:(N-1)*6
    Current_global(i-1,1)= V_current(count)*Current_dir(1);
    Current_global(i,1)    = V_current(count)*Current_dir(2);   
    count=count+1;
end
% end Nodes
Current_global(1,1)=V_current(1)*Current_dir(1);
Current_global(2,1)=V_current(1)*Current_dir(2);
Current_global(N*6-2,1)=V_current(count)*Current_dir(1);
Current_global(N*6-1,1)=V_current(count)*Current_dir(2);
Current_global(N*6,1)=0;

%% local Current Vector
Current_local=(R_g)'*Current_global;
end

