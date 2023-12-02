function [C_RB,C_A] = Coriolis(M_rov,M_rov_a,Vr)
%% Renaming matrices
M=M_rov;
A=M_rov_a;

%% divide translational velocity and rotational velocity vectors
TransVel=Vr(1:3);
RotVel=Vr(4:6);
M11= M(1:3,1:3);
M12= M(1:3,4:6);
M21= M12';
M22= M(4:6,4:6);
A11= A(1:3,1:3);
A12= A(1:3,4:6);
A21= A(4:6,1:3);
A22= A(4:6,4:6);

%% Rigid Body
Trans = M11*TransVel  + M12*RotVel;
Rot   = M21*TransVel  + M22*RotVel;

S1=[    0  -Trans(3)   Trans(2)
      Trans(3)     0   -Trans(1)
     -Trans(2)   Trans(1)     0 ];
 
S2=[    0  -Rot(3)   Rot(2)
      Rot(3)     0   -Rot(1)
     -Rot(2)   Rot(1)     0 ];


C_RB=[ zeros(3,3) -S1
   -S1   -S2];

%% Added Mass
TransVel=Vr(1:3);
RotVel=Vr(4:6);
Trans = A11*TransVel  + A12*RotVel;
Rot   = A21*TransVel  + A22*RotVel;

S1=[    0  -Trans(3)   Trans(2)
      Trans(3)     0   -Trans(1)
     -Trans(2)   Trans(1)     0 ];
S2=[    0  -Rot(3)   Rot(2)
      Rot(3)     0   -Rot(1)
     -Rot(2)   Rot(1)     0 ];

C_A=[ zeros(3,3) S1
   S1	S2];   
end
