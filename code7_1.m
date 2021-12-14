clear all;
close all;
A=[0.837 0.289;-0.231 0.318];
B=[0.175;0.618];
C=[1 0]; D=[0];
q=1;k1=1;
M=[1.154 0.332];
xk=[5;-1]; uk=2;
T=0.1;

for k=1:1:50
t(k)=(k-1)*T;
x(:,k)=xk;
u(:,k)=uk;
x1(k)=x(1,k);
x2(k)=x(2,k);
s(:,k)=M*x(:,k);
uk=inv(M*B)*((((1-q*T)*M)-M*A)*x(:,k)-k1*T*sign(M*x(:,k)));
xk=A*xk+B*uk;
end

figure(1);
subplot(2,2,1);
plot(t,x1,'r.-','linewidth',1);
xlabel('Time [seconds]');ylabel('x1');
axis([0 5 -2 6])
grid on;
subplot(2,2,2);
plot(t,x2,'r.-','linewidth',1);
xlabel('Time [seconds]');ylabel('x2');
axis([0 5 -2 1])
grid on;
subplot(2,2,3);
plot(t,s,'r.-','linewidth',1);
xlabel('Time [seconds]');ylabel('s');
axis([0 5 -2 6])
grid on;
subplot(2,2,4);
stairs(t,u,'r.-','linewidth',1);
xlabel('Time [seconds]');ylabel('u');
axis([0 5 -1 3])
grid on;