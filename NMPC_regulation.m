%%
clc;
clear;
close all;
%% Defining Parameters and Kinematics Equations
Ts = 0.01; %sampling time
Tf = 4 ;
t = 0:Ts:Tf ;
n=3; %number of states
m=2; %number of inputs
N = numel(t);
Np = 10 ; %prediction horizon
d=1; %distance between wheels
r=1; %radius of the wheels
x = zeros(n , N) ; x(:,1)=[0,0,0]; %states
u = zeros(m , N) ; % control inputs (v,omega)
u1= zeros(m , N) ; % control inputs (phi_r dot, phi_l dot)
y = zeros(n , N) ; % outputs

A = eye(n);
B = @(x) [cos(x(3)) 0;sin(x(3)) 0;0 1] ;
C = [1/r d/(2*r) ; 1/r -d/(2*r)] ;
f = @(x , u) A*x+Ts*B(x)*u ; % system kinematics
Uopt = zeros(m , Np); % optimal control inputs
%% Reference Signal
Yref=[3*ones(1,N+Np);5*ones(1,N+Np)];
%% Controller Algorithm
LB = -5*ones(m , Np); % control inputs lower bound
UB = 5*ones(m , Np); % control inputs upper bound
options = optimoptions('fmincon','Display','off','Algorithm','active-set');
for i =1:N-1
    FCN = @(U) Cost_r(x(: ,i) , U , Yref(:,i+1:i+Np) , Np , Ts) ; % cost function
    Uopt = fmincon(FCN , Uopt,[],[],[],[],LB , UB , [] , options);
    u(:,i) = Uopt(:,1); % receding horizon strategy
    u1(:,i) = C*u(:,i);
    x(: , i+1) = f(x(: , i) , u(:,i)); % updating states
end

%% Plots and Animation
figure(1)
subplot(2,1,1)
plot(t,u(1,:),t,u(2,:),'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('u(t)')
legend('v(t)','\omega(t)')

subplot(2,1,2)
plot(t,u1(1,:),t,u1(2,:),'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('u(t) (rad/s)')
legend('d\phi_r(t)/dt','d\phi_l(t)/dt')

figure(2)
subplot(2,2,[3 4])
plot(x(1,:),x(2,:),'linewidth',1.5)
grid on
xlabel('x(t) (m)')
ylabel('y(t) (m)')

subplot(2,2,1)
plot(t,x(1,:),t,Yref(1,1:N),'--','linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('x(t) (m)')
legend('x(t)','Reference')

subplot(2,2,2)
plot(t,x(2,:),t,Yref(2,1:N),'--','linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('y(t) (m)')
legend('y(t)','Reference')

figure(3);
plot(x(1,:),x(2,:),'k--');
grid on
title('workspace')
xlabel('x(t) (m)')
ylabel('y(t) (m)')
hold on;
wls=[0 0;-0.25*d 0.25*d];
wlsrot=[cos(x(3,i)) -sin(x(3,i));sin(x(3,i)) cos(x(3,i))]*wls;
h1=plot(wlsrot(1,1)+x(1,1),wlsrot(2,1)+x(2,1),'ro','LineWidth',2,'MarkerFaceColor','r');
h2=plot(wlsrot(1,2)+x(1,1),wlsrot(2,2)+x(2,1),'ro','LineWidth',2,'MarkerFaceColor','r');
h3=plot(x(1,1),x(2,1),'bo','MarkerSize',20);

for i=2:N
    wlsrot=[cos(x(3,i)) -sin(x(3,i));sin(x(3,i)) cos(x(3,i))]*wls;
    set(h1,'XData',wlsrot(1,1)+x(1,i));
    set(h1,'YData',wlsrot(2,1)+x(2,i));
    set(h2,'XData',wlsrot(1,2)+x(1,i));
    set(h2,'YData',wlsrot(2,2)+x(2,i));
    set(h3,'XData',x(1,i));
    set(h3,'YData',x(2,i));
    
    drawnow;
    pause(Ts);
end