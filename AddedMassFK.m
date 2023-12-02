function  [A_g] = AddedMassFK(A_w,rho,D,l,N)
A_g=zeros(N*6,1);
F_a=zeros(N+1,3);
%% Added mass and Froude Krilov forces
for i=2:N
    F_a(i,2)=0.5*rho*2*pi*((D^2)/4)*(A_w(i-1,2)+A_w(i,2))*l;
    F_a(i,3)=0.5*rho*2*pi*((D^2)/4)*(A_w(i-1,3)+A_w(i,3))*l;
end

%% End Nodes
    % 1st Node
    F_a(1,2)  = 0.5*rho*2*pi*((D^2)/4)*(A_w(1,2))*l;
    F_a(1,3)  = 0.5*rho*2*pi*((D^2)/4)*(A_w(1,3))*l;
    % Last Node
    F_a(N+1,3)= 0.5*rho*2*pi*((D^2)/4)*(A_w(N,3))*l;
    F_a(N+1,2)= 0.5*rho*2*pi*((D^2)/4)*(A_w(N,2))*l;

    
%% Assemble global vector
count=2;
    for i=6:6:(N-1)*6
        A_g(i-2,1)   =  F_a(count,2);
        A_g(i-1,1)   =  F_a(count,3);
        count=count+1;
    end
% Boundary Nodes
    A_g(1,1)     =F_a(1,2);
    A_g(2,1)     =F_a(1,3);    
    A_g((N)*6-2,1)     =F_a(N+1,2);
    A_g((N)*6-1,1)     =F_a(N+1,3);

end

