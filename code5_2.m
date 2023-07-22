function main
clear all;
close all;
global A B C D
A=[0 1 0;0 0 1;-10 -9 -6];
B=[0;0;1];
C=[1 0 0];D=[0];
K=[20 22 4];
M=[1 1 1];
xk=[10;5;-5];
zk=[10;5;-5];
uk=0;uk0=0; 
T=0.001;

for k=1:1:5000
t(k)=k*T;
x1r(k)=0;
time=[0 T];
[tt,xx]=ode45(@(t,x) sysdis(t,x,uk),time,xk);
xk=xx(length(xx),:)'; 
x1(k)=xk(1);
x2(k)=xk(2);
x3(k)=xk(3);
[tt,zz]=ode45(@(t,z) sysnom(t,z,uk0),time,zk);
zk=zz(length(zz),:)'; 
z1(k)=zk(1);
z2(k)=zk(2);
z3(k)=zk(3);
u0(k)=-K*xk; % Equivalent control/ Nominal control
s(k)=M*(xk-zk);
ud(k)=-5*sign(s(k)); % Discontinuous control
u(k)=u0(k)+ud(k);
uk=u(k);
uk0=u0(k);
end

figure(1);
subplot(2,2,1);
plot(t,x1,'r',t,x2,'g',t,x3,'k','linewidth',2);
xlabel('Time [seconds]');ylabel('States');
legend('x1','x2','x3');
axis([0 5 -40 40])
grid on;
subplot(2,2,2);
plot(t,z1,'r',t,z2,'g',t,z3,'k','linewidth',2);
xlabel('Time [seconds]');ylabel('States: Nominal');
legend('z1','z2','z3');
axis([0 5 -40 40])
grid on;
subplot(2,2,3);
plot(t,s,'r','linewidth',2);
xlabel('Time [seconds]');ylabel('Sliding variable');
axis([0 5 -2 2])
grid on;
subplot(2,2,4);
plot(t,u,'r','linewidth',2);
xlabel('Time [seconds]');ylabel('Control input');
axis([0 5 -300 100]);
grid on;

function dx=sysnom(t,x,u)
global A B
dx=zeros(3,1);

dx=A*x+B*u;

function dx=sysdis(t,x,u)
global A B
dx=zeros(3,1);
d=4*sin(t);

dx=A*x+B*u+B*d;
