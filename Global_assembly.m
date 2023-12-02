function [K_g,M_g,C_g] = Global_assembly(K_e,M_e,R_g,N,alpha,beta)
%%*************************************************************************
% This function calculates the stiffness, inertia and proportional damping*
% matrices in the global reference frame.                                 *
%%*************************************************************************

%preallocate zero matrices
R=eye(9,9);
K_g=zeros(N*6,N*6);
M_g=zeros(N*6,N*6);
%% FIRST ELEMENT ----------------------------------------------------------
% Transformation from global to local frame
for i=1:3
    for j=1:3
        R(i,j)=R_g(i,j);
        R(i+3,j+3)=R_g(i,j);
        R(6+i,6+j)=R_g(6+i,6+j);
    end
end
% stiffness and mass matrix of first element in local frame
M_e_int=M_e;
K_e_int=K_e;
% Apply boundary condition for first node (no moment)
K_e_int(:,4:6)=[]; K_e_int(4:6,:)=[];
M_e_int(:,4:6)=[]; M_e_int(4:6,:)=[];
% Transform stiffness and inertia from local to global frame
M_e_int=R*M_e_int*R';
K_e_int=R*K_e_int*R';
% Assemble entries into global matrices
for i=1:9
    for j=1:9
        K_g(i,j)=K_e_int(i,j);
        M_g(i,j)=M_e_int(i,j);
    end
end

%% Intermediate elements --------------------------------------------------
% Transformation from global to local frame
 R=eye(12,12);
for i=6:6:(N-2)*6

    for j=1:6
        for k=1:6
            R(j,k)=R_g(i-3+j,i-3+k);
            R(j+6,k+6)=R_g(i-3+j,i-3+k);
        end
    end
    % Transform stiffness and inertia from local to global frame
    M_e_int=R*M_e*R';
    K_e_int=R*K_e*R';
    % Assemble entries into global matrices
    for j=1:12
        for k=1:12
            K_g(i-3+j,i-3+k)=K_g(i-3+j,i-3+k)+K_e_int(j,k);
            M_g(i-3+j,i-3+k)=M_g(i-3+j,i-3+k)+M_e_int(j,k);
        end
    end
end
%% Last element -----------------------------------------------------------
% Preallocate
R=eye(9,9);
M_e_int=M_e;
K_e_int=K_e;

% Apply boundary condition for last node (no moment)
K_e_int(:,10:12)=[];K_e_int(10:12,:)=[];
M_e_int(:,10:12)=[];M_e_int(10:12,:)=[];

% Transformation matrix from global to local element frame
for i=1:3
    for j=1:3
        R(i,j)=R_g((N-1)*6-3+i,(N-1)*6-3+j);
        R(i+6,j+6)=R_g((N-1)*6-3+i,(N-1)*6-3+j);
    end
end
% Transform stiffness and inertia from local to global frame
M_e_int=R*M_e_int*R';
K_e_int=R*K_e_int*R';
% Assemble entries into global matrices
for j=1:9
    for k=1:9
        K_g((N-1)*6-3+j,(N-1)*6-3+k)= K_g((N-1)*6-3+j,(N-1)*6-3+k)+K_e_int(j,k);
        M_g((N-1)*6-3+j,(N-1)*6-3+k)  = M_g((N-1)*6-3+j,(N-1)*6-3+k)+M_e_int(j,k);
    end
end
%% Proportional Damping(rayleigh damping)
C_g=K_g*alpha+M_g*beta;
end


