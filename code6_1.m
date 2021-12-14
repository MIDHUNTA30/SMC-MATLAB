function main
clear all;
close all;
xk=[0;0];
zk=[-.5;0];
uk=0; sk=0;
T=0.001;

for k=1:1:5000
t(k)=k*T;
x1r(k)=2+1*sin(k*T);
x2r(k)=1*cos(k*T);
time=[0 T];
[tt,xx]=ode45(@(t,x) pendulum(t,x,uk),time,xk,uk);
xk=xx(length(xx),:)'; 
x1(k)=xk(1);
x2(k)=xk(2);
s(k)=x1r(k)-x1(k);

[tt,zz]=ode45(@(t,z) smo(t,z,sk),time,zk,sk); % Estimating s dot
zk=zz(length(zz),:)'; 
z1(k)=zk(1);
z2(k)=zk(2);
u(k)=15*sign(z1(k))+5*sign(z2(k));
uk=u(k);
sk=s(k);
end

figure(1);
plot(x1r-x1,x2r-x2,'r','linewidth',2);
xlabel('s');ylabel('s dot');
grid on;
figure(2);
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
plot(t,z2,'r','linewidth',2)
xlabel('time(s)');ylabel('Estimation of s dot');
axis([0 5 -10 10]);
grid on;
subplot(2,2,4);
plot(t,u,'r','linewidth',2)
xlabel('time(s)');ylabel('Control signal');
axis([0 5 -40 40]);
grid on;

function dx=pendulum(t,x,u)
dx=zeros(2,1);
g=9.8;M=1;B=1;l=1;
dt=5*sin(t);

dx(1)=x(2);
dx(2)=-(g/l)*sin(x(1))-(B/M*l^2)*x(2)+(1/M*l^2)*u+dt;

function dz=smo(t,z,s);
dz=zeros(2,1);

dz(1)=-10*abs((z(1)-s)^0.5)*sign(z(1)-s)+z(2);
dz(2)=-30*sign(z(1)-s);