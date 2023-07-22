function main
clear all;
close all;
xk=[0;0;0]; uk=[0;0]; vk=[0;0];
T=0.001;
for k=1:1:2*pi*1000
t(k)=k*T;
x1r(k)=1*cos(k*T);
x1rd(k)=-1*sin(k*T);
x2r(k)=1*sin(k*T);
x2rd(k)=1*cos(k*T);
x3r(k)=(pi/2)+1*k*T;
x3rd(k)=1;
vr(k)=(x1rd(k)^2+x2rd(k)^2)^.5;
time=[0 T];
[tt,xx]=ode45(@(t,x) kinematic(t,x,uk),time,xk);
xk=xx(length(xx),:)'; 
x1(k)=xk(1);
x2(k)=xk(2);
x3(k)=xk(3);
x1e(k)=(x1r(k)-x1(k))*cos(x3(k))+(x2r(k)-x2(k))*sin(x3(k));
x2e(k)=-(x1r(k)-x1(k))*sin(x3(k))+(x2r(k)-x2(k))*cos(x3(k));
x3e(k)=x3r(k)-x3(k);
u1d(k)=vr(k)*cos(x3e(k))+5*x1e(k);
u2d(k)=10*vr(k)*x2e(k)+x3rd(k)+10*vr(k)*sin(x3e(k));
[tt,uu]=ode45(@(t,u) dynamic(t,uk,vk),time,uk);
uk=uu(length(uu),:)'; 
u1(k)=uk(1);
u2(k)=uk(2);
s1(k)=u1(k)-u1d(k);
s2(k)=u2(k)-u2d(k);
v1(k)=-50*sign(s1(k));
v2(k)=-50*sign(s2(k));
uk=[u1(k);u2(k)];
vk=[v1(k);v2(k)];
end
figure(1)
plot(x1r,x2r,'k',x1,x2,'r','linewidth',2)
xlabel('x1');ylabel('x2');
legend('Reference','Responce');
axis([-1.5 1.5 -1.5 1.5]);
grid on;
figure(2);
subplot(2,2,1);
plot(t,x1r,'k',t,x1,'r','linewidth',2);
xlabel('Time [seconds]');ylabel('x1');
legend('x1 reference','x1 response');
axis([0 2*pi -2 2]);
grid on;
subplot(2,2,2);
plot(t,x1e,'r',t,x2e,'k',t,x3e,'g','linewidth',2);
xlabel('Time [seconds]');ylabel('Tracking errors');
legend('x1 error','x2 error','x3 error');
axis([0 2*pi -1 2]);
grid on;
subplot(2,2,3);
plot(t,s1,'r',t,s2,'k','linewidth',2);
xlabel('Time [seconds]');ylabel('Sliding variable');
legend('s1','s2');
axis([0 2*pi -15 5]);
grid on;
subplot(2,2,4);
plot(t,u1,'r',t,u2,'k','linewidth',2);
xlabel('Time [seconds]');ylabel('Control input');
legend('u1','u2');
axis([0 2*pi -2 10]);
grid on;

function dx=kinematic(t,x,u)
dx=zeros(3,1);

dx(1)=u(1)*cos(x(3));
dx(2)=u(1)*sin(x(3));
dx(3)=u(2);

function du=dynamic(t,u,v)
du=zeros(2,1);
M=10;J=.55;B=.05;D=.01;

du(1)=-(B/M)*u(1)+(1/M)*v(1);
du(2)=-(D/J)*u(2)+(1/J)*v(2);
