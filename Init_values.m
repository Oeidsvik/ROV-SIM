function [u_0,u_f,v,a,P_g,R_g,Result,Results_ROV,...
    Thrust,CableForces,ResultCounter,Weight]...
    = Init_values( N,l,Write_frequency,T,Weight_C)
%% Pre allocate  matrix
R_g=zeros((N+1)*6,(N+1)*6);

%% initial position vectors 
u_0=zeros(N*6+6,1);                         % (u_0=undeformed cable deflection)
for i=6:6:(N+1)*6
    u_0(i-3)=-l*(i/6-1);
end
u_f=zeros(N*6+6,1);                         %(u_f=elastic cable deflection)
v=zeros(N*6+6,1);                           % cable velocity v=v_f+v_0
a=v;                                        % cable acceleration a=a_f+a_0
%% Defining zero matrices
P_g=zeros((N+1)*6,1);
%% Defining initial Transformation matrix (90 deg rotation)
R1=[0,0,1,0,0,0;0,1,0,0,0,0;-1,0,0,0,0,0;0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1];
R_i=[R1,zeros(6,6);zeros(6,6),R1];
%% Assembling initial transformation matrix
for i=12:6:(N+1)*6   
    for j=1:12
        for k=1:12   
            R_g(i+1-j,i+1-k) = R_i(13-j,13-k);
        end
    end
end
%% Preallocation Result outputs
Result=zeros((T+1)*Write_frequency,(N)*6);
Thrust=zeros((T+1)*Write_frequency,5);
CableForces=zeros((T+1)*Write_frequency,4);
ResultCounter=1;

%% Global cable weight vector
Weight=zeros(N*6,1);
Weight(3,1)=Weight_C*l/2;
Weight(N*6,1)=Weight_C*l/2;
for i=5:6:((N-1)*6)
    Weight(i+1,1)=(Weight_C*l);
end
Results_ROV=zeros(size(Result,1),6);
 end