MINV=inv(M);              %Inverse Matric M
C=MINV*A;

a = flip(C(1:5,1));
b = flip(C(6:9,1));
c = flip(C(10:14,1));

%Define local time for each segment 
t1=[0:0.1:2];        
t2=[2:0.1:6];
t3=[6:0.1:8];

%Compute the position equation for each segment
y1=polyval(a,(0:0.1:2));
y2=polyval(b,(0:0.1:4));
y3=polyval(c,(0:0.1:2));

%Define and compute the velocity equation for each segment
a_1 = polyder(a);              
b_1 = polyder(b);
c_1 = polyder(c);

y1_v = polyval(a_1,[0:0.1:2]);
y2_v = polyval(b_1,[0:0.1:4]);
y3_v = polyval(c_1,[0:0.1:2]);

%Define and compute the Acceleration equation for each segment
a_2 = polyder(a_1);
b_2 = polyder(b_1);
c_2 = polyder(c_1);

y1_a = polyval(a_2,[0:0.1:2]);
y2_a = polyval(b_2,[0:0.1:4]);
y3_a = polyval(c_2,[0:0.1:2]);

% plot position
figure(1)
hold on;
plot(t1,y1,'b');
plot(t2,y2,'k');
plot(t3,y3,'r');
title('Angle Profile of End Effector');
xlabel('Time(s)');
ylabel('θ(°)');
hold off;

% plot velocity
figure(2);
hold on;
plot(t1,y1_v,'b');
plot(t2,y2_v,'k');
plot(t3,y3_v,'r');
title('Velocity Profile of End Effector');
xlabel('Time(s)');
ylabel('V(m/s)');
hold off;

%plot acceleration
figure(3);
hold on;
plot(t1,y1_a,'b');
plot(t2,y2_a,'k');
plot(t3,y3_a,'r');
title('Acceleration Profile of End Effector');
xlabel('Time(s)');
ylabel('A(m/s^2)');
hold off;