function J = Cost_t(xi,U,Yref,Np,Ts)
%%% U = [u(i) u(i+1)   u(i+Nc-1)]';
n=3;
m=2;
d=1;
r=1;
A = eye(n);
B =@(x) [cos(x(3)) 0;sin(x(3)) 0;0 1];
f = @(x , u) A*x+Ts*B(x)*u ;

x = zeros(n , Np) ;
x = [xi x] ;
y = zeros(n , Np) ;
for i = 1:Np
    x(: , i+1) = f(x(: , i) , U(:,i));
    y(:,i) = x(: , i+1);
end
R =0.01*eye(m);
Q=[1 0;0 1];
J=0;
for i=1:Np
    J=J+(y(1:2,i)-Yref(1:2,i))'*Q*(y(1:2,i)-Yref(1:2,i))+(U(:,i)'*R*U(:,i));
end
end
