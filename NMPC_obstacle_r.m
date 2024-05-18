%%
clc;
clear;
close all;
%% Defining Parameters and Kinematics Equations
Ts = 0.01;
Tf = 10 ;
t = 0:Ts:Tf ;
n=3;
m=2;
N = numel(t);
Np = 10 ;
d=1;
r=1;
x = zeros(n , N) ; x(:,1)=[0,0,0];
u = zeros(m , N) ;
u1= zeros(m , N) ;
y = zeros(n , N) ;

A = eye(n);
B = @(x) [cos(x(3)) 0;sin(x(3)) 0;0 1] ;
C = [1/r d/(2*r) ; 1/r -d/(2*r)] ;
f = @(x , u) A*x+Ts*B(x)*u ;
Uopt = zeros(m , Np);
%% Reference Signal
Yref=[30*ones(1,N+Np);30*ones(1,N+Np)];
%% Obstacles
Ob1=zeros(n,N);
Xo1=5;
Yo1=5;
Ro1=1;
Ob1(3,:)=linspace(0,4*pi,N);
for i=1:N
    Ob1(1:2,i)=[Xo1+Ro1*cos(Ob1(3,i));Yo1+Ro1*sin(Ob1(3,i))];
end

Ob2=zeros(n,N);
Xo2=20;
Yo2=19;
Ro2=0.9;
Ob2(3,:)=linspace(0,4*pi,N);
for i=1:N
    Ob2(1:2,i)=[Xo2+Ro2*cos(Ob2(3,i));Yo2+Ro2*sin(Ob2(3,i))];
end
%% Controller Algorithm
LB = -5*ones(m , Np);
UB = 5*ones(m , Np);
options = optimoptions('fmincon','Display','off','Algorithm','active-set');
for i =1:N-1
    FCN = @(U) Cost_obstacle_r(x(: ,i) , U , Yref(:,i+1:i+Np) , Np , Ts) ;
    CNS = @(U) nlc_r(x(: ,i) , U , Np , Ts, Xo1 , Yo1 ,Ro1, Xo2 , Yo2 ,Ro2) ;
    Uopt = fmincon(FCN , Uopt,[],[],[],[],LB , UB , CNS , options);
    u(:,i) = Uopt(:,1);
    u1(:,i) = C*u(:,i);
    x(: , i+1) = f(x(: , i) , u(:,i));
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
plot(x(1,:),x(2,:),Ob1(1,:),Ob1(2,:),'--',Ob2(1,:),Ob2(2,:),'--','linewidth',1.5)
grid on
xlabel('x(t) (m)')
ylabel('y(t) (m)')
legend('Mobile Robot Trajectory','Obstacle1','Obstacle2')

subplot(2,2,1)
plot(t,x(1,:),'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('x(t) (m)')

subplot(2,2,2)
plot(t,x(2,:),'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('y(t) (m)')