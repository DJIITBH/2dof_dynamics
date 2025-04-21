close all;
J=0.824;
b=0.0079;
L=0.0163;
k=1.3908;
R=1.7;
m1 = 0.25;
m2 = 0.25;
l1 = 1;
l2 = 1;


kp_1=30.0;
ki_1=2.0;
kd_1=15.0;

kp_2=20.0;
ki_2=2.0;
kd_2=15.0;


% Time period of controller(50 ms)
dt = 0.01;

% % Total simulation time
% t_total = 0.2;

% tspan = 0:dt:t_total; % Full time range

y0 = zeros(6, 1); % Initial state

V=0.0 ;  % Initial control input

y_all = [];       % To store all states
t_all = [];       % To store all time


current_state = y0; % Start with the initial state

% Declaring Temp variables;
error_prev_theta1=0;
error_prev_theta1_in=0;

error_prev_theta2=0;
error_prev_theta2_in=0;

%************
x0=1; y0=1;
xf=-1; yf=0;
%initial & final end-effector position
xdot0=0; ydot0=0;
xdotf=0; ydotf=0;
tf=1;      %trajectory duration
t=0:0.1:tf; %time span
L1=1;L2=1;
%Joint positions using IK

[th10,th20] = inverse_kinematics(x0,y0,1,1);
[th1f,th2f] = inverse_kinematics(xf,yf,1,1);

J0= [-L1*sin(th10)-L2*sin(th10+th20),-L2*sin(th10+th20);
    +L1*cos(th10)+L2*cos(th10+th20),+L2*cos(th10+th20);]
Jf= [-L1*sin(th1f)-L2*sin(th1f+th2f),-L2*sin(th1f+th2f);
    +L1*cos(th1f)+L2*cos(th1f+th2f),+L2*cos(th1f+th2f);]
qdot0=inv(J0)*[xdot0; ydot0];   %Initial joint velocity
qdotf=inv(Jf)*[xdotf; ydotf];   %Final joint velocity

A= [1,0,0,0;
    0,1,0,0;
    1,tf,tf^2,tf^3;
    0,1,2*tf,3*tf^2];

b1= [th10;qdot0(1);th1f;qdotf(1)];
b2= [th20;qdot0(2);th2f;qdotf(2)];
tc1= inv(A)*b1;
tc2= inv(A)*b2;
% th1=[0.25,0.5,0.75,1.0,1.25];
% th2=[0.25,0.5,0.75,1.0,1.25];
for j=1:length(t)
    %Joint space trajectory
    th1(j)= [1,t(j),t(j)^2,t(j)^3]*tc1;
    th2(j)= [1,t(j),t(j)^2,t(j)^3]*tc2;

    theta1 = th1(j);
    theta2 = th2(j);
% motor control loop 
    tspan = (1.2)*(j-1):dt:(1.2)*(j); % Full time range

    for i = 1:length(tspan)-1
    
        t_current = [tspan(i), tspan(i+1)];
    
         [V1,error_prev_theta1,error_prev_theta1_in]=pid_control(current_state(1),theta1,error_prev_theta1,error_prev_theta1_in,kp_1,ki_1,kd_1,dt);
       
         [V2,error_prev_theta2,error_prev_theta2_in]=pid_control(current_state(2),theta2,error_prev_theta2,error_prev_theta2_in,kp_2,ki_2,kd_2,dt);
        if V1>24
        V1=24;
        end
        if V1<-24
            V1=-24;
        end
    
        if V2>24
            V2=24;
        end
        if V2<-24
            V2=-24;
        end  
        % Solve ODE for this time step
        [t1, fx] = ode45(@(t1, fx) robot_dynamics(t1,fx,J,b,L,k,R,V1,V2,m1,m2,l1,l2), t_current, current_state);
        
        % Store all the results
        y_all = [y_all; fx(1:end-1, :)]; % Exclude the last point to avoid duplication
        t_all = [t_all; t1(1:end-1)];
    
        % Update the current state of the variables
        current_state = fx(end, :)';
        [xe,ye] = forward_kinematics(fx(end,1), fx(end,2), 1, 1);
        plot([0 l1*cos(fx(end,1)) xe], [0 l1*sin(fx(end,1)) ye], 'b-o' )
    
    
    end
end


y_all = [y_all; current_state'];
t_all = [t_all; tspan(end)];
skip = 200;

 % **********
for i = 1:skip:length(y_all)
    theta1 = y_all(i,1);
    theta2 = y_all(i,2);

    % Elbow point
    x1 = l1 * cos(theta1);
    y1 = l1 * sin(theta1);

    % End-effector
    [xe, ye] = forward_kinematics(theta1, theta2, l1, l2);

    plot([0 x1 xe], [0 y1 ye], 'b-o', 'LineWidth', 2)
    hold on
    plot(x0, y0, 'rs', 'markersize', 10);  % Start
    plot(xf, yf, 'gp', 'markersize', 10);  % Goal
    axis([-(l1+l2) (l1+l2) -(l1+l2) (l1+l2)]);
    axis square
    grid on
    set(gca,'fontsize',12,'fontname','Times');
    xlabel('x [units]');
    ylabel('y [units]');
    % drawnow;
    pause(0.05)
    hold off
end

% 
figure;
plot(t_all, y_all(:, 1), 'r');
grid on;
xlabel('Time (s)'); ylabel('Theta1');
title('Theta');


figure;
plot(t_all, y_all(:, 2), 'g');
grid on;
xlabel('Time (s)'); ylabel('Theta2');
title('Theta');

figure;
plot(t_all, y_all(:, 3), 'b');
grid on;
xlabel('Time (s)'); ylabel('omega1');
title('Omega');

figure;
plot(t_all, y_all(:, 4), 'r');
grid on;
xlabel('Time (s)'); ylabel('omega2');
title('Omega');


figure;
plot(t_all, y_all(:, 5), 'g');
grid on;
xlabel('Time (s)'); ylabel('acc1');
title('Acc');

figure;
plot(t_all, y_all(:, 6), 'b');
grid on;
xlabel('Time (s)'); ylabel('acc2');
title('Acc');

