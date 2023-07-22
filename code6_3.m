function main
clear all;
close all;
xk=[0;0;0];
zk=[0;0;0];
uk=0; sk=0;
T=0.001;

for k=1:1:5000
t(k)=k*T;
x1r(k)=2+1*sin(k*T);
time=[0 T];
[tt,xx]=ode45(@(t,x) thirdorder(t,x,uk),time,xk);
xk=xx(length(xx),:)'; 
x1(k)=xk(1);
x2(k)=xk(2);
x3(k)=xk(3);
s(k)=x1r(k)-x1(k);

[tt,zz]=ode45(@(t,z)smo2(t,z,sk),time,zk);
zk=zz(length(zz),:)'; 
z1(k)=zk(1);
z2(k)=zk(2);
z3(k)=zk(3);
u(k)=20*sign(z3(k)+2*(((abs(z2(k)))^3+(abs(z1(k)))^2)^(1/6))...
*sign(z2(k)+((abs(z1(k)))^(2/3))*sign(z1(k))));
uk=u(k);
sk=s(k);
end

figure(1);
subplot(2,2,1);
plot(t,x1r,'k',t,x1,'r','linewidth',2);
xlabel('Time [seconds]');ylabel('x1');
legend('x1 reference','x1 response');
axis([0 5 0 5]);
grid on;
subplot(2,2,2);
plot(t,s,'k',t,z1,'r','linewidth',2);
xlabel('time(s)');ylabel('Sliding variable');
legend('s','Estimation of s');
axis([0 5 -1 4]);
grid on;
subplot(2,2,3);
plot(t,z2,'r',t,z3,'k','linewidth',2)
xlabel('time(s)');ylabel('Estimation of $\dot{s}$,$\ddot{s}$','Interpreter','latex');
leg1 = legend('Estimation of $\dot{s}$','Estimation of $\ddot{s}$');
set(leg1,'Interpreter','latex');
axis([0 5 -20 20]);
grid on;
subplot(2,2,4);
plot(t,u,'r','linewidth',2)
xlabel('time(s)');ylabel('Control signal');
axis([0 5 -40 40]);
grid on;

function dx=thirdorder(t,x,u)
dx=zeros(3,1);
dt=5*sin(t);

dx(1)=x(2);
dx(2)=x(3);
dx(3)=-2*sin(x(1))-3*cos(x(2))-dt+u;

function dz=smo2(t,z,s)
dz=zeros(3,1);
v1=-10*(abs(z(1)-s)^(2/3))*sign(z(1)-s)+z(2);
v2=-20*(abs(z(2)-v1)^.5)*sign(z(2)-v1)+z(3);

dz(1)=v1;
dz(2)=v2;
dz(3)=-100*sign(z(3)-v2);
