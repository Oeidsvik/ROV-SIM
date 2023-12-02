function [W_v,W_a] = Waves(k,u,h,H,N,t,Wave_dir)

W_a=zeros(N*6,1);
W_v=zeros(N*6,1);

if k==0 % no waves
else
    % Wave Frequency
    omega=k*9.81*tanh(k*h);
    % Global wave velocity and acceleration at each node + Stokes Drift
    for i=5:6:N*6-1
        W_v(i-1,1)=cosd(Wave_dir)*(omega)*(H/2)*cosh(k*(u(i+1)+h))/sinh(k*h)*sin(omega*t-k*cosd(Wave_dir)*u(i-1)-k*sind(Wave_dir)*u(i))...
            + cosd(Wave_dir)*(H/2)^2*omega*k*cosh(2*k*(u(i+1)+h))/(2*(sinh(k*h))^2);
        W_v(i,1)=sind(Wave_dir)*(omega)*(H/2)*cosh(k*(u(i+1)+h))/sinh(k*h)*sin(omega*t-k*cosd(Wave_dir)*u(i-1)-k*sind(Wave_dir)*u(i))...
            + sind(Wave_dir)*(H/2)^2*omega*k*cosh(2*k*(u(i+1)+h))/(2*(sinh(k*h))^2);
        W_v(i+1,1)=(omega)*(H/2)*sinh(k*(u(i+1)+h))/sinh(k*h)*cos(omega*t-k*cosd(Wave_dir)*u(i-1)-k*sind(Wave_dir)*u(i));
        W_a(i-1,1)=cosd(Wave_dir)*(omega)^2*(H/2)*cosh(k*(u(i+1)+h))/sinh(k*h)*cos(omega*t-k*cosd(Wave_dir)*u(i-1)-k*sind(Wave_dir)*u(i));
        W_a(i,1)=sind(Wave_dir)*(omega)^2*(H/2)*cosh(k*(u(i+1)+h))/sinh(k*h)*cos(omega*t-k*cosd(Wave_dir)*u(i-1)-k*sind(Wave_dir)*u(i));
        W_a(i+1,1)=-(omega)^2*(H/2)*sinh(k*(u(i+1)+h))/sinh(k*h)*sin(omega*t-k*cosd(Wave_dir)*u(i-1)-k*sind(Wave_dir)*u(i));
    end
    
    % 1st Node
    % Velocity
    W_v(1,1)=cosd(Wave_dir)*(omega)*(H/2)*cosh(k*(u(3)+h))/sinh(k*h)*sin(omega*t-k*cosd(Wave_dir)*u(1)-k*sind(Wave_dir)*u(2))...
        + cosd(Wave_dir)*(H/2)^2*omega*k*cosh(2*k*(u(3)+h))/(2*(sinh(k*h))^2);
    W_v(2,1)=sind(Wave_dir)*(omega)*(H/2)*cosh(k*(u(3)+h))/sinh(k*h)*sin(omega*t-k*cosd(Wave_dir)*u(1)-k*sind(Wave_dir)*u(2))...
        + sind(Wave_dir)*(H/2)^2*omega*k*cosh(2*k*(u(3)+h))/(2*(sinh(k*h))^2);
    W_v(3,1)=(omega)*(H/2)*sinh(k*(u(3)+h))/sinh(k*h)*cos(omega*t-k*cosd(Wave_dir)*u(1)-k*sind(Wave_dir)*u(2));
    % Acceleration
    W_a(1,1)=cosd(Wave_dir)*(omega)^2*(H/2)*cosh(k*(u(3)+h))/sinh(k*h)*cos(omega*t-k*cosd(Wave_dir)*u(1)-k*sind(Wave_dir)*u(2));
    W_a(2,1)=sind(Wave_dir)*(omega)^2*(H/2)*cosh(k*(u(3)+h))/sinh(k*h)*cos(omega*t-k*cosd(Wave_dir)*u(1)-k*sind(Wave_dir)*u(2));
    W_a(3,1)=-(omega)^2*(H/2)*sinh(k*(u(3)+h))/sinh(k*h)*sin(omega*t-k*cosd(Wave_dir)*u(1)-k*sind(Wave_dir)*u(2));
    
end
