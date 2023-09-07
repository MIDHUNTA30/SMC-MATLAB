function main
clear all;
close all;
xk=[2;0]; uk=0;
c=1; T=0.001;
for k=1:1:5000
t(k)=k*T;
x1r(k)=0;
x1rd(k)=0;
time=[0 T];
tk=t(k);
[tt,xx]=ode45(@(t,x) massspring(tk,xk,uk),time,xk);
xk=xx(length(xx),:); 
x1(k)=xk(1);
x2(k)=xk(2);
s(k)=x2(k)+c*x1(k);
u(k)=5*sign(s(k));
uk=u(k);
end
figure(1);
subplot(2,2,1);
plot(t,x1,'r',t,x2,'k','linewidth',2);
xlabel('Time [seconds]');ylabel('States');
legend('x1','x2');
axis([0 5 -2 3])
grid on;
subplot(2,2,2);
plot(t,s,'r','linewidth',2);
xlabel('Time [seconds]');ylabel('Sliding variable');
axis([0 5 -1 3])
grid on;
subplot(2,2,3);
x=linspace(-4,10,100);
y=-c*x;
plot(x,y,'k','linewidth',2);
hold on;
plot(x1,x2,'r','linewidth',2);
xlabel('x1');ylabel('x2');
legend('sliding surface','state trajectory');
axis([-2 2 -2 2]);
grid on;
subplot(2,2,4);
plot(t,u,'r','linewidth',2);
xlabel('Time [seconds]');ylabel('Control input');
axis([0 5 -10 10])
grid on;

function dx=massspring(t,x,u)
dx=zeros(2,1);
r=0;
dx(1)=x(2);
dx(2)=-x(1)+r-u;
