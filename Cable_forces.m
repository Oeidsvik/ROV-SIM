function [Rayleigh,Restoring,Inertia_inv] = Cable_forces(Rayleigh,...
    Restoring,Inertia,v,u_f,N)
%% Calculate Cable Forces
Rayleigh= (Rayleigh*v);
Restoring=(Restoring*(u_f));
Inertia_inv=(Inertia)\eye(N*6,N*6);
end

