%%Modified from online resources
%%Denavit-Hatenberg Parameters
% i  thi  ai   alphai    di
% 1  th1   0    pi/2     l1
% 2  th2  l2       0      0
% 3  th3  l3       0      0

syms th1 th2 th3 l1 l2 l3;
% distance from one joint to the other, units in meter 
l1 = 0.225; 
l2 = 0.60596; 
l3 = 0.55298; 

% Transformation matrix
AR= [1 0 0 0; 
     0 1 0 0;
     0 0 1 l1;
     0 0 0 1];
 
A1= [cos(th1) 0 sin(th1) 0;
     sin(th1) 0 -cos(th1) 0;
     0 1 0 l1;
     0 0 0 1];
 
A2= [cos(th2) -sin(th2) 0 l2*cos(th2);
     sin(th2) cos(th2) 0 l2*sin(th2);
     0 0 1 0;
     0 0 0 1];
 
A3= [cos(th3) -sin(th3) 0 l3*cos(th3);
     sin(th3) cos(th3) 0 l3*sin(th3);
     0 0 1 0;
     0 0 0 1];

% Transformation from base to each link
AR1=A1;
AR2=AR1*A2;
AR3=AR2*A3;

% Links Specifications, obtained from solidworks
% Mass of the links in kg [assume density is 1000kg/m^3)
m1=4.96180; 
m2=8.60068; 
m3=6.52663;

% position of centre of mass [x,y,z,0], obtained from solidworks, (assuming
% everything is positioned upright
r1=[0; 0; 0.03570; 0];
r2=[-0.00002; 0; 0.38557; 0];
r3=[0.00001; 0; 0.73673; 0];

   %Uij Matrixes, zero if the term does not contain the angle
   U11=diff(AR1,th1);   U21=diff(AR2,th1); U31=diff(AR3,th1);
   U12=zeros(4);        U22=diff(AR2,th2); U32=diff(AR3,th2);
   U13=zeros(4);        U23=zeros(4);      U33=diff(AR3,th3);
  
   %Uijk Matrixes
   %i = 1
   %U1jk Matrixes
   U111=diff(U11,th1); U121=zeros(4)     ; U131=zeros(4);
   U112=zeros(4)     ; U122=zeros(4)     ; U132=zeros(4);
   U113=zeros(4)     ; U123=zeros(4)     ; U133=zeros(4);

   %i = 2
   %U2jk Matrixes
   U211=diff(U21,th1); U221=diff(U22,th1); U231=zeros(4);
   U212=diff(U21,th2); U222=diff(U22,th2); U232=zeros(4);
   U213=zeros(4)     ; U223=zeros(4)     ; U233=zeros(4);

   %i = 3
   %U3jk Matrixes
   U311=diff(U31,th1); U321=diff(U32,th1); U331=diff(U33,th1);
   U312=diff(U31,th2); U322=diff(U32,th2); U332=diff(U33,th2);
   U313=diff(U31,th3); U323=diff(U32,th3); U333=diff(U33,th3);
          

%Moment of Inertia matrixes obtain from Solidworks (kgm^2)
  I1=[0.02533          0           0;
            0    0.02497           0;
            0          0     0.03283];
  
  I2=[0.31601          0           0;
            0    0.31464           0;
            0          0     0.01815];

  I3=[0.14230           0          0;
            0     0.14307          0;
            0           0    0.01620];

  %Construction of J matrix (Pseudo inertia matrix)
  I=zeros(4);
  for i=1:3
      I=eval(['I' num2str(i)]);
      m=eval(['m' num2str(i)]);
      r=eval(['r' num2str(i)]);
      eval(['j11' '=((-I(1,1)+I(2,2)+I(3,3))/2)']);
      eval(['j12' '=I(1,2)']);
      eval(['j13' '=I(1,3)']);
      eval(['j14' '=m*r(1)']);
      
      eval(['j21' '=I(1,2)']);
      eval(['j22' '=((I(1,1)-I(2,2)+I(3,3))/2)']);
      eval(['j23' '=I(2,3)']);
      eval(['j24' '=m*r(2)']);
      
      eval(['j31' '=I(1,3)']);
      eval(['j32' '=I(2,3)']);
      eval(['j33' '=((I(1,1)+I(2,2)-I(3,3))/2)']);
      eval(['j34' '=m*r(3)']);
      
      eval(['j41' '=m*r(1)']);
      eval(['j42' '=m*r(2)']);
      eval(['j43' '=m*r(3)']);
      eval(['j44' '=m']);
      
      J=[j11 j12 j13 j14;
         j21 j22 j23 j24; 
         j31 j32 j33 j34;
         j41 j42 j43 j44];
      eval(['J' num2str(i) '=J']);
  end
  
 %Construction of D matrix or Dij 
  Uaux=zeros(4);
for i=1:3
   for j=1:3
       m=max([i j]);
       x=0;
       for k=m:3 %p in the formula
           Uaux=eval(['U' num2str(k) num2str(i)]);
           Ud=Uaux';
           A=eval(['U' num2str(k) num2str(j)])*eval(['J' num2str(k)])*Ud;
           x=x+trace(A);
       end
   eval(['d' num2str(i) num2str(j) '=x']);
   end
end

D=[d11 d12 d13 ;
   d21 d22 d23 ;
   d31 d32 d33 ];

%Dijk matrix 
  for i=1:3
   for k=1:3
     for m=1:3
       j=max([i m k]);
       x=0;
       for l=j:3
       Uaux=eval(['U' num2str(j) num2str(i)]);
       Uh=Uaux'; %transpose
       x=x+trace(eval(['U' num2str(j) num2str(k) num2str(m)])*eval(['J' num2str(j)])*Uh);
       end
       eval(['h' num2str(i) num2str(k) num2str(m) '=x']);
     end 
   end
  end
  
   %Coriolis column matrix
 syms dth1 dth2 dth3 %angular velocity
 
  for i=1:3
      y=0;
      for k=1:3
        y=y+x;
        x=0;  
          for m=1:3
          y=x+eval(['h' num2str(i) num2str(k) num2str(m)])*eval(['dth' num2str(k)])*eval(['dth' num2str(m)]);
          end    
      end
  eval(['h' num2str(i) '=y']);
  end
  
  H=[h1;h2;h3]; 
  
    %Gravity column matrix  
  g=[0 0 -9.81 0];
  
  for i=1:3
      x=0;
      for j=i:3
      x=x+(-eval(['m' num2str(j)])*g*eval(['U' num2str(j) num2str(i)])*eval(['r' num2str(j)]));  
      end
  eval(['c' num2str(i) '=x']);
  end
   
  C=[c1;
     c2;
     c3];
  
  %Final dynamic mode
  %angular acceleration matrix
  
  syms ddth1 ddth2 ddth3 
  ddth=[ddth1;
        ddth2;
        ddth3]; 
  %T=[t1;t2;t3;t4;t5;t6;t7]; %Matrix with torques of each joint
  
  T=D*ddth+H+C;

 
  

  
