function main
clear all;
close all;
k1=10; k2=10;
xk=[10;-5;10]; uk=[0;0];
T=0.001;

for k=1:1:5000
t(k)=k*T;
x1r(k)=0;
x2r(k)=0;
time=[0 T];
[tt,xx]=ode45(@(t,x) mimo(t,x,uk),time,xk);
xk=xx(length(xx),:)'; 
x1(k)=xk(1);
x2(k)=xk(2);
x3(k)=xk(3);
s1(k)=x1(k)+x2(k);
s2(k)=x1(k)+x2(k)+x3(k);
u1(k)=-x2(k)-x3(k)-k1*sign(s1(k));
u2(k)=-x2(k)-x3(k)-u1(k)-k2*sign(s2(k));
uk=[u1(k);u2(k)];
end

figure(1);
A=1;B=1;C=0;D=0;
[xa za] = meshgrid(-10:0.1:10);
ya = -1/B*(A*xa + C*za + D);  
A=1,B=1,C=1,D=0;
[xb yb] = meshgrid(-10:0.1:10);
zb = -1/C*(A*xb + B*yb + D); 
plot3(x1,x2,x3,'b',xa,ya,za,'y',xb,yb,zb,'g');
axis([-10 10 -10 10 -10 10])
grid on
rotate3d on
xlabel('x1');ylabel('x2');zlabel('x3');
text(-5,10,5,'s1=0');
text(5,10,5,'s2=0');
figure(2);
subplot(2,2,1);
plot(t,x1,'r',t,x2,'k',t,x3,'g','linewidth',2);
xlabel('Time [seconds]');ylabel('States');
legend('x1','x2','x3');
axis([0 5 -8 12]);
grid on;
subplot(2,2,2);
plot(t,s1,'r',t,s2,'k','linewidth',2);
xlabel('Time [seconds]');ylabel('Sliding variables');
legend('s1','s2');
axis([0 5 -5 15]);
grid on;
subplot(2,2,3);
plot(t,u1,'r','linewidth',2);
xlabel('Time [seconds]');ylabel('Control input 1');
axis([0 5 -20 20]);
grid on;
subplot(2,2,4);
plot(t,u2,'r','linewidth',2);
xlabel('Time [seconds]');ylabel('Control input 2');
axis([0 5 -30 10]);
grid on;

function dx=mimo(t,x,u)
dx=zeros(3,1);
d1=0.5*sin(t);
d2=0.5*sin(t);

dx(1)=x(2);
dx(2)=x(3)+d1+u(1);
dx(3)=d2+u(2);
