function main
clear all;
close all;
global A B C D M K L 
A=[0 1 0;0 0 1;-10 -9 -6];
B=[0 1;1 0;0 -1];
C=[1 0 0;1 1 1];D=[0 0;0 0];
K=[5;10];M=[0.5 1 0.5;0.5 0 -0.5]; L=[1 0;1 -1;-2 2]; K2=[10;20];
xk=[5;-5;1]; zk=[4;-4;0];
uk=[0;0];vk=[0;0];
T=0.001;

for k=1:1:5000
t(k)=k*T;
x1r(k)=0;
x2r(k)=0;
time=[0 T];
[tt,xx]=ode45(@(t,x) linear(t,x,uk),time,xk);
xk=xx(length(xx),:)'; 
x1(k)=xk(1);
x2(k)=xk(2);
x3(k)=xk(3);
x(:,k)=xk;
[tt,zz]=ode45(@(t,z) smolinear(t,z,uk,vk),time,zk);
zk=zz(length(zz),:)'; 
z1(k)=zk(1);
z2(k)=zk(2);
z3(k)=zk(3);
z(:,k)=zk;
s(:,k)=M*z(:,k);
u(:,k)=-inv(M*B)*M*A*z(:,k)-inv(M*B)*(K.*sign(s(:,k)));
s2(:,k)=C*(x(:,k)-z(:,k));
v(:,k)=K2.*sign(s2(:,k));
uk=[u(1,k);u(2,k)];
vk=[v(1,k);v(2,k)];
end

figure(1);
subplot(2,2,1);
plot(t,x1,'r',t,x2,'k',t,x3,'g','linewidth',2);
hold on;
plot(t,z1,'r',t,z2,'k',t,z3,'g','linewidth',2);
xlabel('Time [seconds]');ylabel('States');
leg1 = legend('x1 and $\hat{x1}$','x2 and $\hat{x2}$','x3 and $\hat{x3}$');
set(leg1,'Interpreter','latex');
axis([0 5 -5 5])
grid on;
subplot(2,2,2);
plot(t,s(1,:),'r',t,s(2,:),'k','linewidth',2);
xlabel('Time [seconds]');ylabel('Sliding variables');
legend('s1','s2');
axis([0 5 -2 2])
grid on;
subplot(2,2,3)
plot(t,u(1,:),'r','linewidth',2);
axis([0 5 -20 20])
xlabel('Time [seconds]');ylabel('Control input 1');
grid on;
subplot(2,2,4)
plot(t,u(2,:),'r','linewidth',2);
axis([0 5 -20 20])
xlabel('Time [seconds]');ylabel('Control input 2');
grid on;

function dx=linear(t,x,u)
global A B
dx=zeros(3,1);

dx=A*x+B*u;

function dz=smolinear(t,z,u,v)
global A B L
dz=zeros(3,1);

dz=A*z+B*u+L*v;
