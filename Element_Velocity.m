function [Vr] =Element_Velocity(Current_local, v,W_v,N)

%% Element relative-velocity to flow (mean of nodal velocity)
count=2;
Vr=zeros(N,3);
for i=12:6:(N-1)*6
    Vr(count,1) = (-W_v(i-2)-Current_local(i-2)+v(i-2))/2 ...
                 +(-W_v(i-8)-Current_local(i-8)+v(i-8))/2; 
             
    Vr(count,2) = (-W_v(i-1)  -Current_local(i-1)+v(i-1))/2 ...
                + (-W_v(i-7)-Current_local(i-7)+v(i-7))/2; 
            
    Vr(count,3) = (-W_v(i)-Current_local(i)+v(i))/2 ...
                + (-W_v(i-6)-Current_local(i-6)+v(i-6))/2; 
   
    count=count+1;
end
% end Nodes
Vr(1,1)       = (-W_v(4)-Current_local(4)+v(4))/2 ...
               +(-W_v(1)-Current_local(1)+v(1))/2;
            
Vr(1,2)       =  (-W_v(5)-Current_local(5)+v(5))/2 ...
                +(-W_v(2)-Current_local(2)+v(2))/2;
              
Vr(1,3)       =  (-W_v(6)-Current_local(6)+v(6))/2 ...
                 +(-W_v(3)-Current_local(3)+v(3))/2;          
          
Vr(N,1)       = - (-W_v(N*6-2)-Current_local(N*6-2)+v(N*6-2))/2 ...
               + -(-W_v(N*6-8)-Current_local(N*6-8)+v(N*6-8))/2;

Vr(N,2)       =  (-W_v(N*6-1)-Current_local(N*6-1)+v(N*6-1))/2 ...
              +  (-W_v(N*6-7)-Current_local(N*6-7)+v(N*6-7))/2;

Vr(N,3)       =  (-W_v(N*6)-Current_local(N*6)+v(N*6))/2 ...
              +  (-W_v(N*6-6)-Current_local(N*6-6)+v(N*6-6))/2;

end

