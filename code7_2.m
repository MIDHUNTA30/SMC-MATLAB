clear all;
close all;
A1=[0.837 0.289;-0.231 0.318];
B1=[0.175;0.612];
A2=[0.930 0.190;-0.152 0.589];
B2=[0.052;0.390];
C=[1 0];D=[0];
M=[1.154 0.332];
xk=[5;-1];uk=2;y=[0;0];
k1=1; q=1;
T=0.1; N=2;
Co=[C;C*A2];
Do=[0;C*B2];
Ly=A1*inv(Co);
Lu=B1-inv(Co)*Do;
Fy=inv(M*B1)*(((1-q*T)*M)-M*A1)*Ly;
Fu=inv(M*B1)*(((1-q*T)*M)-M*A1)*Lu;

for k=1:1:50
t(k)=(k-1)*T;
x(:,k)=xk;
u(:,k)=uk;
x1(k)=xk(1);
x2(k)=xk(2);

yk=xk;
for m=1:1:N  
y(m)=C*yk; % Multi rate output
yk=A2*yk+B2*uk;
end

s(:,k)=M*x(:,k);
uk=Fy*y+Fu*uk-inv(M*B1)*k1*T*sign(s(:,k));
xk=A1*xk+B1*uk;
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