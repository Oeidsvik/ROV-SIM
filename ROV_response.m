    function [v_rov,a_rov,u_rov] = ROV_response(v_rov,...
        Current_global,N,R_rov,Tau_body,M_rov,...
        M_rov_a,D_rov_quad,D_rov_lin,g_eta,Tension_ROV,...
        u_rov,dt,a_rov,Weight_C,l,Cable_rov_drag)
   
%%                           Description:
%**************************************************************************
%     This function has the cable tension and thrust forces as inputs. It
%     then estimates the resulting rigid body acceleration of the ROV in
%     the body fixed frame. The acceleration is then converted to the
%     global reference frame. The boundary condition for
%     the last element is then given from the rigid body acceleration.
%     Newmark beta time integration is further used to estimate velocity
%     and position of the ROV. The translational motion is not used since
%     it will be found based on the complete cable system later. However
%     the rotational motions must be estimated since these states are kept
%     independent of the cable motion.
%**************************************************************************    

    
    %% Current relative velocity-------------------------------------------
    Current_ROV=zeros(6,1);
    Current_ROV(1:3,1)=Current_global(N*6-2:N*6,1);
    V_relative_rov=R_rov'*(v_rov-Current_ROV);
  
    %% Cable weight of last cable node(last cable element)
    Cable_weight_rov_global=zeros(6,1);
    Cable_weight_rov_global(3,1)=-Weight_C*l/2;
    Cable_weight_rov_local=R_rov*Cable_weight_rov_global;
     
    %% Cable drag of last cable node( last element/2)
    Cable_rov_drag=[Cable_rov_drag',0,0,0]';
    Cable_rov_drag=R_rov'*Cable_rov_drag;
    
    %% Coriolis and centripetal forces and moments
    [C_RB,C_A]=Coriolis(M_rov,M_rov_a,V_relative_rov);
%C_RB=C_RB*0;
%C_A=C_A*0;
    %% Acceleration of ROV in body-----------------------------------------
    a_rov_body_new=(M_rov_a+M_rov)\(Tension_ROV-g_eta+Tau_body+Cable_weight_rov_local-...
    D_rov_lin*V_relative_rov-D_rov_quad*V_relative_rov.*abs(V_relative_rov)-C_RB*V_relative_rov-C_A*V_relative_rov-Cable_rov_drag);        
    %% Convert acceleration to global frame--------------------------------
    a_rov_new=R_rov*a_rov_body_new;
    
    %% find velocity and position (newmark-beta)
      v_rov_new= v_rov +...
        (1-0.5)*dt*a_rov + 0.5*dt*a_rov_new  ;
        u_rov_new=u_rov+dt*v_rov+...
        (0.25)*dt^2*a_rov+0.25*dt^2*a_rov_new;


    %% Update acceleration, velocity and position for next step
    % NED( North East Down)
    u_rov=u_rov_new;
    a_rov=a_rov_new;
    v_rov=v_rov_new;


    end

