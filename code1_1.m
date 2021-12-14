% main function
function main
clear all;
close all;
% initilization
xk=[1;0];
time=[0 10];
% solve using ode45
[tt,xx]=ode45(@(t,x) autonomous(t,x),time,xk);
x1=xx(:,1);
x2=xx(:,2);
% plotting the responce
plot(x1,x2,'r','linewidth',2);
hold on;
xlabel('x1');ylabel('x2');
axis([-1 1 -1 1])
grid on;

% state equations: autonomous system
function dx=autonomous(t,x)
%initialization
dx=zeros(2,1);
% define elements of x dot vector
dx(1)=x(2);
dx(2)=-x(1);
