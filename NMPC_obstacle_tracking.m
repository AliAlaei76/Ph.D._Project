%%
clc;
clear;
close all;
%% Defining Parameters and Deriving Kinematics Equations
Ts = 0.01;
Tf = 10 ;
t = 0:Ts:Tf ;
n=3;
m=2;
N = numel(t);
Np = 35 ;
d=1;
r=1;
x = zeros(n , N) ; x(:,1)=[0,0,0];
u = zeros(m , N) ;
u1= zeros(m , N) ;
y = zeros(n , N) ;

A = eye(n);
B =@(x) [cos(x(3)) 0;sin(x(3)) 0;0 1] ;
C = [1/r d/(2*r) ; 1/r -d/(2*r)] ;
f = @(x , u) A*x+Ts*B(x)*u ;
Uopt = zeros(m , Np);
%% Generating desired trajectory
Yref=zeros(n,N);
xc=0;
yc=0;
Rr=2;
Yref(3,:)=linspace(0,4*pi,N);
for i=1:N
    Yref(1:2,i)=[xc+Rr*cos(Yref(3,i));yc+Rr*sin(Yref(3,i))];
end
Yref=[Yref Yref(:,1:Np)];
%% Obstacles
Ob1=zeros(n,N);
Xo1=-0.5;
Yo1=2;
Ro1=0.3;
Ob1(3,:)=linspace(0,4*pi,N);
for i=1:N
    Ob1(1:2,i)=[Xo1+Ro1*cos(Ob1(3,i));Yo1+Ro1*sin(Ob1(3,i))];
end

Ob2=zeros(n,N);
Xo2=0;
Yo2=-1.8;
Ro2=0.4;
Ob2(3,:)=linspace(0,4*pi,N);
for i=1:N
    Ob2(1:2,i)=[Xo2+Ro2*cos(Ob2(3,i));Yo2+Ro2*sin(Ob2(3,i))];
end
%% Controller Algorithm
LB = -5*ones(m , Np);  
UB = 5*ones(m , Np);  
options = optimoptions('fmincon','Display','off','Algorithm','active-set');
for i =1:N-1
    FCN = @(U) Cost_obstacle_t(x(: ,i) , U , Yref(:,i+1:i+Np) , Np , Ts) ;
    CNS = @(U) nlc_t(x(: ,i) , U , Np , Ts, Xo1 , Yo1 ,Ro2, Xo2 , Yo2 ,Ro2) ;
     Uopt = fmincon(FCN , Uopt,[],[],[],[],LB , UB , CNS , options);
    u(:,i) = Uopt(:,1);
    u1(:,i) = C*u(:,i);
    x(: , i+1) = f(x(: , i) , u(:,i));
end

%% Plots and Animation
figure(1)
subplot(2,1,1)
plot(t(1:N-1),u(1,1:N-1),t(1:N-1),u(2,1:N-1),'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('u(t)')
legend('v(t)','\omega(t)')

subplot(2,1,2)
plot(t(1:N-1),u1(1,1:N-1),t(1:N-1),u1(2,1:N-1),'linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('u(t) (rad/s)')
legend('d\phi_r(t)/dt','d\phi_l(t)/dt')

figure(2)
subplot(2,2,[3 4])
plot(x(1,:),x(2,:),Yref(1,1:N),Yref(2,1:N),'--',Ob1(1,:),Ob1(2,:),'--',Ob2(1,:),Ob2(2,:),'--','linewidth',1.5)
grid on
xlabel('x(t) (m)')
ylabel('y(t) (m)')
legend('Mobile Robot Trajectory','Desired Trajectory','Obstacle1','Obstacle2')

subplot(2,2,1)
plot(t,x(1,:),t,Yref(1,1:N),'--','linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('x(t) (m)')
legend('x(t)','x_d(t)')

subplot(2,2,2)
plot(t,x(2,:),t,Yref(2,1:N),'--','linewidth',1.5)
grid on
xlabel('Time (s)')
ylabel('y(t) (m)')
legend('y(t)','y_d(t)')