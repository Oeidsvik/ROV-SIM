function [u_ship,v_ship,a_ship] = Surface_ship(Ship_desired)
    %% Position
    u_ship(1:6)=Ship_desired(1:6);
     %% velocity
     v_ship(1:6)=zeros(1,6);
     %% acceleration
     a_ship(1:6)=zeros(1,6);
end

