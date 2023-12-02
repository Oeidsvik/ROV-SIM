function [] = Plotter(Result,ll,N,E_control_result)
count=1;
for j=5:6:(N)*6
    x1(count+1,1)=Result(round(ll/5),j-1);
    z1(count+1,1)=Result(round(ll/5),j+1);
    x2(count+1,1)=Result(round(2*ll/5),j-1);
    z2(count+1,1)=Result(round(2*ll/5),j+1);
    x3(count+1,1)=Result(round(3*ll/5),j-1);
    z3(count+1,1)=Result(round(3*ll/5),j+1);
    x4(count+1,1)=Result(round(4*ll/5),j-1);
    z4(count+1,1)=Result(round(4*ll/5),j+1);
    x5(count+1,1)=Result(ll-1,j-1);
    z5(count+1,1)=Result(ll-1,j+1);
    
    count=count+1;
end
figure(1)
x1(1,1)=Result(round(ll/5),1);
z1(1,1)=Result(round(ll/5),3);
z1(count,1)=Result(round(ll/5),N*6);
x2(1,1)=Result(round(2*ll/5),1);
z2(1,1)=Result(round(2*ll/5),3);
z2(count,1)=Result(round(2*ll/5),N*6);
x5(1,1)=Result(ll,1);
z5(1,1)=Result(ll,3);
z5(count,1)=Result(round(5*ll/5),N*6);
x3(1,1)=Result(round(3*ll/5),1);
z3(1,1)=Result(round(3*ll/5),3);
z3(count,1)=Result(round(3*ll/5),N*6);
x4(1,1)=Result(round(4*ll/5),1);
z4(1,1)=Result(round(4*ll/5),3);
z4(count,1)=Result(round(4*ll/5),N*6);
subplot(2,2,1);
plot(x1,z1);
hold on
plot(x2,z2);
hold on
plot(x3,z3);
hold on
plot(x4,z4);
hold on
plot(x5,z5);
hold on
title('Umbilical deflection North')
legend(['T=',num2str(round(ll/5))],...
    ['T=',num2str(round(2*ll/5))],...
    ['T=',num2str(round(3*(ll)/5))],...
    ['T=',num2str(round(4*(ll)/5))],...
    ['T=',num2str(ll)])
xlabel('Horisontal Deflection North[m]')
ylabel('Depth[m]')


count=1;
for j=5:6:(N)*6
    y1(count+1,1)=Result(round(ll/5),j);
    z1(count+1,1)=Result(round(ll/5),j+1);
    y2(count+1,1)=Result(round(2*ll/5),j);
    z2(count+1,1)=Result(round(2*ll/5),j+1);
    y3(count+1,1)=Result(round(3*ll/5),j);
    z3(count+1,1)=Result(round(3*ll/5),j+1);
    y4(count+1,1)=Result(round(4*ll/5),j);
    z4(count+1,1)=Result(round(4*ll/5),j+1);
    y5(count+1,1)=Result(ll,j);
    z5(count+1,1)=Result(ll,j+1);
    
    count=count+1;
end

y1(1,1)=Result(round(ll/5),2);
z1(1,1)=Result(round(ll/5),3);
z1(count,1)=Result(round(ll/5),N*6);

y2(1,1)=Result(round(2*ll/5),2);
z2(1,1)=Result(round(2*ll/5),3);
z2(count,1)=Result(round(2*ll/5),N*6);
y5(1,1)=Result(ll,2);
z5(1,1)=Result(ll,3);
z5(count,1)=Result(round(5*ll/5),N*6);
y3(1,1)=Result(round(3*ll/5),2);
z3(1,1)=Result(round(3*ll/5),3);
z3(count,1)=Result(round(3*ll/5),N*6);
y4(1,1)=Result(round(4*ll/5),2);
z4(1,1)=Result(round(4*ll/5),3);
z4(count,1)=Result(round(4*ll/5),N*6);

subplot(2,2,2);
plot(y3,z3);
hold on
plot(y1,z1);
hold on
plot(y2,z2);
hold on
plot(y4,z4);
hold on
plot(y5,z5);
hold on
legend(['T=',num2str(round(ll/5))],...
    ['T=',num2str(round(2*ll/5))],...
    ['T=',num2str(round(3*(ll)/5))],...
    ['T=',num2str(round(4*(ll)/5))],...
    ['T=',num2str(ll)])
xlabel('Horisontal Deflection East[m]')
ylabel('Depth[m]')
title('Umbilical deflection East')
subplot(2,2,3);
plot3(x3,y3,z3);
hold on
plot3(x1,y1,z1);
hold on
plot3(x2,y2,z2);
hold on
plot3(x4,y4,z4);
hold on
plot3(x5,y5,z5);
hold on
xlabel('Horisontal Deflection North[m]')
ylabel('Horisontal Deflection East[m]')
zlabel('Depth[m]')
title('Umbilical Deflection NED')


figure(2)
subplot(2,2,1)
plot(E_control_result(:,1),E_control_result(:,2))
hold on
xlabel('Time [s]')
ylabel('Error[m/s]')
title('Surge speed Error')
hold on
subplot(2,2,2)
plot(E_control_result(:,1),E_control_result(:,4))
hold on
xlabel('Time [s]')
ylabel('Error[rad]')
title('heading Error')
hold off
subplot(2,2,3)
plot(E_control_result(:,1),E_control_result(:,6))
hold on
xlabel('Time [s]')
ylabel('Error[m]')
title('Depth Error')
hold off

end

