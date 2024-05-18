function [C , Ceq] = nlc_t(xi,U,Np,Ts,Xo1,Yo1,Ro1,Xo2,Yo2,Ro2)
%%% U = [u(i) u(i+1)   u(i+Nc-1)]';
n=3;
m=2;

safezone1=Ro1+0.5;
safezone2=Ro2+0.5;

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

C=zeros(2,1);
for i=1:Np
    C(1)=C(1)+safezone1^2-(y(1,i)-Xo1)^2-(y(2,i)-Yo1)^2;
    C(2)=C(2)+safezone2^2-(y(1,i)-Xo2)^2-(y(2,i)-Yo2)^2;
end
Ceq = [] ;
end
