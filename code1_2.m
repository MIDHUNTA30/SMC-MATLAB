% main function
function main
clear all;
close all;
% initialization
xk=[1;0];
uk=0;
T=0.01;
for k=1:1:1000
t(k)=k*T;
time=[0 T];
tk=t(k);
% solve using ode45
[tt,xx]=ode45(@(t,x) nonautonomous(tk,xk,uk),time,xk);
% updating state vector, 
% which acts as the initial state of next iteration.
% last row in xx vector is taken as current state vector.
xk=xx(length(xx),:)'; 
x1(k)=xk(1);
x2(k)=xk(2);
% control law
u(k)=-3*x2(k)-1*x1(k);
% updating control input
uk=u(k);
end
% plotting the responce
figure(1);
plot(x1,x2,'r','linewidth',2); 
hold on;
xlabel('x1');ylabel('x2');
axis([-1 1 -1 1])
grid on;

% state equations: non-autonomous system
function dx=nonautonomous(t,x,u)
%initialization
dx=zeros(2,1);
% define elements of x dot vector
dx(1)=x(2);
dx(2)=-x(1)+u;


