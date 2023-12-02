function [E_length] = Element_length(u,N)
%% Element orientation in global(for use in drag calculations)
E_length=zeros(N,1);
Delta_pos=zeros(N,3);
%% First Element
Delta_pos(1,1) = (u(4)- u(1));
Delta_pos(1,2) = (u(5)- u(2));
Delta_pos(1,3) = (u(6)- u(3));
%% length rigid body
Delta_pos(1,1) = (u(4)- u(1));
E_length(1,1)=sqrt(Delta_pos(1,1)^2+Delta_pos(1,2)^2+Delta_pos(1,3)^2);
for i=12:6:(N)*6
 %% element length
    Delta_pos(i/6,1) = (u(i-2) - u(i-8));
    Delta_pos(i/6,2) = (u(i-1) - u(i-7));
    Delta_pos(i/6,3) = (u(i)- u(i-6));
    E_length(i/6,1)=   sqrt(Delta_pos(i/6,1)^2+Delta_pos(i/6,2)^2+Delta_pos(i/6,3)^2);  
end

end

