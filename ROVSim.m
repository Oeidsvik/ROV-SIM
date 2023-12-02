%*************************************************************************
%*************************************************************************
%**                     ROV-Cable simulator                             **
%**                     Linear Beam Theory with FFRF                    **
%**.....................................................................**
%**.....................................................................**
%**                  Version 8.1          12.10.2018                    **
%**                      By: Ole Alexander Eidsvik                      **
%*************************************************************************
%*************************************************************************
close all
clear all
clc
%% Input variables
% Environment -------------------------------------------------------------
rho=1025;                   % Density of water                     [kg/m^3]
g=9.81;                     % Constant of gravity                  [m/s^2]
V_wind=10;                  % Wind velocity 10 m above surface     [m/s]
V_tide=0.4;                 % Tidal current velocity               [m/s]
Current_dir0=45;            % Wind and current direction rel to X  [deg]
Wave_dir=0;                 % Wave direction rel to X              [deg]
h=501;                      % Water depth                          [m]
h_w=50;                     % Reference depth for wind (DNV)       [m]
H=4.5;                      % Wave Height                          [m]
Tw=0;                       % Wave Period                          [s]

% Umbilical/ ROV-----------------------------------------------------------
D= 0.023;                   % Width of umbilical                   [m]
L=500;                      % Length of umbilical                  [m]
M=rho*(pi/4)*D^2;           % Mass per unit length                 [kg/m]
M_A=rho*(pi/4)*D^2;         % Added mass per unit length           [kg/m]
N=25;                       % Number of elements                   [-]
E=1*10^9;                   % Module of elasticity                 [Pa]
A=pi*(D/2)^2;               % Crossection area                     [m^2]
alpha=0.02;                 % Rayleigh stiffness factor(1->10%)    [-]
beta=0.01;                  % Rayleigh mass factor (DNV->0)        [-]
l=L/N;                      % Element Length                       [m]
I=0.25*pi*(D/2)^4*[0,1,1];  % Area moment of inertia               [m^4]
Ct=0.02;                    % Tangential Drag Coefficient          [-]
Init_ROV_yaw=0;             % Initial ROV yaw Angle(heading)       [rad]
Weight_C=-0.1;              % Submerged weight of cable            [N/m]
ROV='Sf30k';

% Controller
Controller_type='PID';      % Types are PID or MB (model-based)

% ROV data-----------------------------------------------------------------
[D_rov_quad,D_rov_lin,M_rov,M_rov_a,BG_rov...
    ,W_rov,B_rov,UG_rov,C_gains,T_limits]= ROV_specifications(g,ROV,Controller_type);

% Simulator Properties ----------------------------------------------------
T=200;                     % Simulation Time                       [s]
T2=T;                      % Time Variable for plotting            [s]
dt=0.005;                  % Integration Step Size                 [s]
Write_frequency=10;        % The frequency of result saving        [Hz]
Ramp_up=50;                % Time before ROV-Path is executed      [s]
% Current Direction vector-------------------------------------------------
Current_dir=[cosd(Current_dir0),sind(Current_dir0)];
%**************************************************************************
%% Simulator Variables-----------------------------------------------------


%% Initial values
[u_0,u_f,v,a,P_g,R_g,Result_cable,Result_ROV,...
    Thrust,CableForces,ResultCounter,Weight]...
    = Init_values( N,l,Write_frequency,T,Weight_C);

%% Defining matrices
[K_e] = Element_stiffness(E,I,A,l);
[M_e,gamma] = Element_mass( M,M_A,l);

%% Calculate Wave number
[k]=Wavelength(g,Tw,h);
%% Apply Boundary Conditions
[P_g,u_0,v,R_g,a,ROV_pos] = Boundary_condition(P_g,u_0,...
    u_f,v,a,R_g,N,Init_ROV_yaw);
%% Assemble Global Matrices
[K_g,M_g,C_g] = Global_assembly(K_e,M_e,R_g,N,alpha,beta);

%% Drag data taken from Blevins fig 10.22 / table 10-18
[Cd] = DragCoefficient(D);

%% Define zero vectors and matrices
[a_rov,v_rov,u_rov,disp_count,...
                Dispvector,u,E_control] = Init_arrays(N,T,u_0);

%% Path Generation for ROV (extremely basic reference path)
[rov_ref] = Path(L,T,dt);

%% Ship position
u_ship=[0,0,0,0,0,0]; %  Initial Surface ship position

%for tuning PID
E_control_result=zeros((T)*Write_frequency,7);

%**************************************************************************
%**************************************************************************
%**************************************************************************
%%*********************       SOLVER       ********************************
%**************************************************************************
%**************************************************************************
%**************************************************************************

for step=1:T/dt+1
    % Write basic simulation information to screen
    if (step-1)*dt==Dispvector(disp_count)
        fprintf('Time:')
        disp((step-1)*dt)
        fprintf('Thruster forces: \n')
        disp(Tau_body);
        disp_count=disp_count+1;
    end
    t=(step-1)*dt;                              % current time   [s]
    %% Calculate Transformation matrix, local velocity and undeformed deflection.
    [R_g,u_0,u_f,v_l] =Transformation(N,l,u,v);
    %% Surface ship position, velocity and acceleration.
    % Ship is fixed for now, add ship response in this function
    [u_ship,v_ship,a_ship] =...
        Surface_ship(u_ship);
    %% Element length
    [E_length] =Element_Length(u,N);
    %% Local and global current velocity
    [Current_local,Current_global] = Current(V_wind,V_tide,N,h_w,h,...
                                              u,Current_dir,R_g);
    %% Local Wave velocity and acceleration ( froude-krilov and wave induced added mass)
    [W_v,W_a] = Waves(k,u,h,H,N,t,Wave_dir);
    %% Element relative-velocity to Current and waves (mean nodal velocity)
    [Vr] = Element_Velocity(Current_local,v_l,W_v,N) ;
    %% Average Wave acceleration on elements
    [A_w] = ElementWaveAcc(W_a,N);
    %% Drag Forces
    [P_g,Cable_rov_drag]=Drag(Vr,N,rho,D,E_length,Cd,Ct,R_g);
    %% Froude-krilov and wave induced added mass
    [A_g] = AddedMassFK(A_w,rho,D,l,N);
    %%  Caclulate global matrices in the global frame
    [K_g,M_g,C_g] =...
        Global_assembly(K_e,M_e,R_g,N,alpha,beta);
    %% Calculate restoring forces,proportional damping and inertia
    [Prop_damping,Restoring,Inertia_inv] = Cable_forces(C_g,K_g,M_g,v,u_f,N);
    %% Calculate acceleration for new step
    % Umbilical
    a_new=Inertia_inv*((-Restoring+P_g- Prop_damping+A_g+Weight));
    % Check if next step is ok? If not, stop simulation.
    Error=Error_check(a_new,u_rov);
    if strcmp(Error,'true')
        break;
    end
    %% ROV Transformation and hydrostatic restoring matrices
    [R_rov,g_eta] = Transformation_rov(u_rov,B_rov,W_rov,BG_rov);
    
    %% ROV Tension forces and moments
    [T_global,T_rov] =ROV_tension(R_rov,UG_rov,N,Restoring);
    if strcmp(Controller_type,'PID')
    %% regular PID-controller
      [Tau_NED,Tau_body,E_control] = PID(dt,t,u_rov,R_rov,v_rov,rov_ref,E_control,C_gains,T_limits);    
    elseif strcmp(Controller_type,'MB')
   %% Model Based PD-controller
            [Tau_NED,Tau_body,E_control] = MB_controller(dt,t,u_rov,a_rov,R_rov,v_rov,rov_ref,E_control,C_gains,T_limits,...
                                                      D_rov_lin,D_rov_quad,M_rov,M_rov_a,g_eta,L);
    else
        disp('Suggested ROV controller does not exist');
        break
    end


    %% ROV response
    [v_rov,a_rov,u_rov] = ROV_response(v_rov,Current_global,N,R_rov,...
                          Tau_body,M_rov,M_rov_a,D_rov_quad,D_rov_lin,...
                          g_eta,T_rov,u_rov,dt,a_rov,Weight_C,l,...
                          Cable_rov_drag);
    %% Update boundary conditions for next iteration
    a_new(1:3,1)=a_ship(1,1:3);               % (from input, GPS, IMU etc.)
    a_new(N*6-2:N*6,1)=a_rov(1:3,1);
      
    %% Calculate velocity and position (newmark beta) 
    v_new= v + (1-0.5)*dt*a + 0.5*dt*a_new;
    u_new= u + dt*v +(0.5-0.25)*dt^2*a+(0.25)*dt^2*a_new;
    u_new(N*6-2:N*6)=(u_rov(1:3));
    v_new(N*6-2:N*6)=v_rov(1:3);
    
    %% Update position, acceleration and velocity
    v=v_new;
    u=u_new;
    a=a_new;
    

    %% Write results to result matrices
    if t>0
        if rem(round(t*Write_frequency,5),1) == 0
            Result_ROV(ResultCounter,1)=t;
            Result_ROV(ResultCounter,2:7)=u_rov(1:6);
            Result_cable(ResultCounter,:)=u;
            Thrust(ResultCounter,1)=t;
            Thrust(ResultCounter,2:4)=Tau_NED(1:3);
            Thrust(ResultCounter,5)=Tau_NED(6);
            CableForces(ResultCounter,1)=t;
            CableForces(ResultCounter,2:4)=...
            Restoring(N*6-2:N*6);
            ResultCounter=ResultCounter+1;
            E_control_result(ResultCounter,1)=t;
            E_control_result(ResultCounter,2:7)=E_control;
        end
    end

end

%% Plot Deflections for certain time steps
Plotter(Result_cable,T*10,N,E_control_result)
