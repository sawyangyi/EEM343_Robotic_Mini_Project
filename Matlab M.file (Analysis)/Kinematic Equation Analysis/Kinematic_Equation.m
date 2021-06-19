%Transformation Matrices/Forward Kinematic Equation

syms   L1 L2 L3 Q1 Q2 Q3 Q4 Q5;
alphaa=[90,0,0]; % this is the alpha value for all  the link
a=[0,L2,L3]; % Length of the Link
d=[L1,0,0]; %Offset
Q=[Q1,Q2,Q3]; % joint angle variation

for i=1:3
    switch i
       case 1
            T01= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 2
            T12= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
        case 3
            T23= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
       
    end
end

T01
T12
T23
T03=simplify(T01*T12*T23)



%Jarcobian
syms px py pz;
px=T03(1,4)
py=T03(2,4)
pz=T03(3,4)

%To find dpx
dpx1=diff(px,Q1)            %differenciate px respect to Q1
dpx2=diff(px,Q2)            %differenciate px respect to Q2
dpx3=diff(px,Q3)            %differenciate px respect to Q3
dpx4=diff(px,Q4)            %differenciate px respect to Q4
dpx5=diff(px,Q5)            %differenciate px respect to Q5

%To find dpy
dpy1=diff(py,Q1)            %differenciate py respect to Q1
dpy2=diff(py,Q2)            %differenciate py respect to Q2
dpy3=diff(py,Q3)            %differenciate py respect to Q3
dpy4=diff(py,Q4)            %differenciate py respect to Q4
dpy5=diff(py,Q5)            %differenciate py respect to Q5

%To find dpz
dpz1=diff(pz,Q1)            %differenciate pz respect to Q1
dpz2=diff(pz,Q2)            %differenciate pz respect to Q2
dpz3=diff(pz,Q3)            %differenciate pz respect to Q3
dpz4=diff(pz,Q4)            %differenciate pz respect to Q4
dpz5=diff(pz,Q5)            %differenciate pz respect to Q5

J=[dpx1 dpx2 dpx3 dpx4 dpx5;dpy1 dpy2 dpy3 dpy4 dpy5;dpz1 dpz2 dpz3 dpz4 dpz5]

