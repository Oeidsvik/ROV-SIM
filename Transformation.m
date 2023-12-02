function [R_g,u_0,u_f,v_l] =Transformation(N,l,u,v)
%% TRANSFORMATION 1st Element----------------------------------------------
R_g=zeros(N*6,N*6);         % global transformation matrix
deltaX = +(u(4)-u(1));      
deltaY = +(u(5)-u(2));      
deltaZ = -((u(6)-u(3)));
[R_el] = RotMatrix(deltaX,deltaY,deltaZ);
for j=1:3
    for k=1:3
        R_g(j,k)  = R_el(j,k)  ;
    end
end
%% 2nd element
    deltaX =  +  ( u(10)- u(4) )  ;            
    deltaY =  +  ( u(11) -u(5)   )   ;                
    deltaZ =  - ( (u(12) -u(6))   );
    [R_el] = RotMatrix( deltaX,deltaY,deltaZ);
    for j=1:6
        for k=1:6
            R_g(5-2+j,5-2+k)=R_el(j,k);
        end
    end
%% Intermediate elements  
for i=12:6:(N-1)*6
    deltaX = + (u(i+4)-u(i-2));                 
    deltaY = + ( u(i+5)-u(i-1));   
    deltaZ = -((u(i+6)-u(i)));
    [R_el] = RotMatrix( deltaX,deltaY,deltaZ);
        for j=1:6
            for k=1:6
                R_g(i-3+j,i-3+k)= R_el(j,k);
            end
        end
end
%% LAST Node
 [R_el] = RotMatrix(deltaX,deltaY,deltaZ);
  for j=1:3
      for k=1:3
          R_g(N*6-3+j,N*6-3+k)= R_el(j,k);
      end
  end
u_0=zeros(N*6,1);
%% Unelastic deflection ---------------------------------------------------
u_0(4:6)=R_g(1:3,1:3)*[l,0,0]';
for i=12:6:N*6
u_0(i-2:i)=u_0(i-8:i-6)+R_g(i-8:i-6,i-8:i-6)*[l,0,0]';
end

%% Elastic deflection ---------------------------------------------------
u_f=u-u_0;
%% VELOCITY 
v_l=R_g'*v;                   % local

end 






