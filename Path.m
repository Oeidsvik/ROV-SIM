%Path generation sinus path
function [rov_ref] = Path(L,T,dt)

%% Parameters for sinus path in x-z plane
A_h=30;                           % Ampltiude of horizontal sinus path
A_v=2;                            % Amplitude of vertical sinus path
T_h=190;                          % Period of sinus path horisontal
T_v=190;                          % Period of sinus path vertical
R_up=10;                          % Ramp up before the path is executed
%Define path vector
rov_ref=zeros(T/dt,4);
%% Find desired states
count=1;
for t=0:dt:T
        % we have a ramp up period in the beginning before the execution of the
        % maneuver
    if t<R_up
        rov_ref(count,1)=t;
        % Speed------
        rov_ref(count,2)=(2*pi*A_h/T_h)*t/R_up;
        % Heading----
        rov_ref(count,3)=0;
        % depth
        rov_ref(count,4)=-L+2*A_v*t/R_up  ;
    else
        rov_ref(count,1)=t;
        % Speed------
        rov_ref(count,2)=2*pi*A_h/T_h;
        % Heading----
        rov_ref(count,3)=rov_ref(count-1,3)+2*pi/T_h*dt;
        % depth------
        % we start by driving the ROV two vertical amplitudes up ( to avoid
        % large tension in the cable when performing horizontal maneuver)
        rov_ref(count,4)=-L+2*A_v-A_v*sin((2*pi/T_v)*(t-R_up));
    end
    count=count+1;
    
    
    
    
end
end
