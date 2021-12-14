function main
clear all;
close all;
global A B C D K L
A=[0 1 0;0 -1 5;0 -4 -10];
B=[0;0;10];
C=[1 0 0];D=[0];
K=[1.6 .64 .4];
L=[34;246;-96];
xk=[10;20;5];
zk=[1;1;1];
uk=0;
yek=0;
T=0.001;

for k=1:1:5000
t(k)=k*T;
x1r(k)=0;
time=[0 T];
[tt,xx]=ode45(@(t,x) dcmotor(t,x,uk),time,xk,uk);
xk=xx(length(xx),:)';
x1(k)=xk(1);
x2(k)=xk(2);
x3(k)=xk(3);
x(:,k)=xk;
[tt,zz]=ode45(@(t,z) observer(t,z,uk,yek),time,zk,uk);
zk=zz(length(zz),:)'; 
z1(k)=zk(1);
z2(k)=zk(2);
z3(k)=zk(3);
p=1/(-C*inv(A-B*K)*B);
u(k)=p*x1r(k)-K*xk;
ye(k)=x1(k)-z1(k);
uk=u(k);
yek=ye(k);
end

figure(1);
subplot(2,2,1);
plot(t,x1,'r',t,x2,'k',t,x3,'g','linewidth',2);
xlabel('Time [seconds]');ylabel('States');
legend('x1','x2','x3');
axis([0 5 -40 40])
grid on
subplot(2,2,2);
plot(t,z1,'r',t,z2,'k',t,z3,'g','linewidth',2);
xlabel('Time [seconds]');ylabel('Estimated states');
legend('z1','z2','z3');
axis([0 5 -40 40])
grid on
subplot(2,2,3);
plot(t,x1-z1,'r',t,x2-z2,'k',t,x3-z3,'g','linewidth',2);
xlabel('Time [seconds]');ylabel('Estimation error');
legend('x1 error','x2 error','x3 error');
axis([0 5 -40 40])
grid on
subplot(2,2,4);
plot(t,u,'r','linewidth',2)
xlabel('Time [seconds]');ylabel('Control input');
axis([0 5 -40 0])
grid on

function dx=dcmotor(t,x,u)
global A B 
dx=zeros(3,1);


dx=A*x+B*u;

function dz=observer(t,z,u,v)
global A B L
dz=zeros(3,1);

dz=A*z+B*u+L*v;