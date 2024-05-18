% This function calculates the cost function and is called in each sample
function J = Cost_r(xi,U,Yref,Np,Ts)
n=3;
m=2;
A = eye(n);
B =@(x) [cos(x(3)) 0;sin(x(3)) 0;0 1];
f = @(x , u) A*x+Ts*B(x)*u ; % kinematic equation

x = zeros(n , Np) ;
x = [xi x] ;
y = zeros(n , Np) ;
for i = 1:Np
    x(: , i+1) = f(x(: , i) , U(:,i)); % updating the system based on the optimization variable U over prediction horizon
    y(:,i) = x(: , i+1);
end
R =0.01*eye(m); % control input weighting matrix
Q=[1 0;0 1]; % state weighting matrix
J=0;
for i=1:Np
    J=J+(y(1:2,i)-Yref(:,i))'*Q*(y(1:2,i)-Yref(:,i))+(U(:,i)'*R*U(:,i)); % cost function over prediction horizon
end
end
