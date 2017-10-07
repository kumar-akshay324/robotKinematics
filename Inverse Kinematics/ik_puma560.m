%   AKSHAY KUMAR
%   WPI ID - 842954269
%   EMAIL - akumar5@wpi.edu
%   SYNERGY OF HUMAN AND ROBOT - RBE595 - F17 - 191S
%   QUESTION 1 - ASSIGNMENT 4 - PART 2

% SOLVING FOR THE JOINT ANGLES OF PUMA 560 USING ALGEBRAIC METHOD - INVERSE
% KINEMATICS

%   The T06_mat input matrix is given as
%
%      r11      r12     r13     px
%      r21      r22     r23     py 
%      r31      r32     r33     pz
%       0        0       0       1


% User is supposed to feed the T06 matrix given above, as the input to
% this. Also, since the vales for a2, a3, d3 and d4 are not available to us
% beforehand, so the user is supposed to feed that as well. 

function angles = ik_puma560(T06_mat,a2,a3,d3,d4)

syms t1 t2 t3 t4 t5 t6          % Joint angles intially given symbolic values as variables

px = T06_mat(1,4);
py = T06_mat(2,4);
pz = T06_mat(3,4);

r11 = T06_mat(1,1);
r12 = T06_mat(1,2);
r13 = T06_mat(1,3);
r21 = T06_mat(2,1);
r22 = T06_mat(2,2);
r23 = T06_mat(2,3);
r31 = T06_mat(3,1);
r32 = T06_mat(3,2);
r33 = T06_mat(3,3);

T01 = [cos(t1) -sin(t1) 0 0;sin(t1) cos(t1) 0 0; 0 0 1 0; 0 0 0 1]; 
T12 = [cos(t2) -sin(t2) 0 0;0 0 1 0; -sin(t2) -cos(t2) 0 0; 0 0 0 1];
T23 = [cos(t3) -sin(t3) 0 a2;sin(t3) cos(t3) 0 0; 0 0 1 d3; 0 0 0 1];
T34 = [cos(t4) -sin(t4) 0 a3;0 0 1 d4; -sin(t4) -cos(t4) 0 0; 0 0 0 1];
T45 = [cos(t5) -sin(t5) 0 0;0 0 -1 0; sin(t5) cos(t5) 0 0; 0 0 0 1];
T56 = [cos(t6) -sin(t6) 0 0;0 0 1 0; -sin(t6) -cos(t6) 0 0; 0 0 0 1];

T02 = T01*T12;
T03 = T01*T12*T23;
T04 = T01*T12*T23*T34;
T05 = T01*T12*T23*T34*T45;
T06 = T01*T12*T23*T34*T45*T56;

T13 = T12*T23;
T14 = T12*T23*T34;
T15 = T12*T23*T34*T45;
T16 = T12*T23*T34*T45*T56;

T24 = T23*T34;
T25 = T23*T34*T45;
T26 = T23*T34*T45*T56;

T35 = T34*T45;
T36 = T34*T45*T56;

T46 = T45*T56;

% For t1: inv(T01)*T06 = T12*T23*T34*T45*T56

t1 = atan2d(py, px) - asind(d3/(sqrt(px^2 + py^2)));

s1 = sind(t1);
c1 = cosd(t1);

% For t3 from the solution


t3 = atan2d(d4,a3) - acosd(((px^2 + py^2 + pz^2 -a2^2 -a3^2 - d3^2 - d4^2)/2*a2)/ sqrt(a3^2 + d4^2));

s3 = sind(t3);
c3 = cosd(t3);

% For t2: inv(T03)*T06 = T34*T45*T56

t2 = atan2d(-(a3+a2*c3)*pz - (c1*px + s1*py)*(d4-a2*s3),-(d4-a2*s3)*pz - (a3+a2*c3)*(c1*px + s1*py)) - t3;

s2 = sind(t2);
c2 = cosd(t2);

c23 = c2*c3 - s2*s3;
s23 = s2*c3 + c2*s3;

% For t4 from the previous solution

t4 = atan2d(s1*r13 - c1*r23, c1*c23*r13 + s1*c23*r23 - s23*r33)

c4 = cosd(t4);
s4 = sind(t4);

% For t5: inv(T04)*T06 = T46

t5 = atan2d((r33*s23*c4 - (c1*c23*c4 + s1*s4)*r13 - (s1*c23*c4-c1s4)*r23), -s23*c1*r13 - s1*s23*r23 - c23*r33);

c5 = cosd(t5);
s5 = sind(t5);

% For t6: inv(T05)*T06 = T56

t6 = atan2d((r31*s23*s4 - r21*(s1*c23*s4 + c1*c4) - r11*(c1*c23*s4 - s1*c4)), ((c1*c23*c4 + s1*s4)*c5 - c1*s23*s5)*r11 + r21*((s1*c23*c4 - c1*s4)*c5 - s1*s23*s5) -r31*(s23*c4*c5+c23*s5))

angles = [t1 t2 t3 t4 t5 t6];   % Complete joint angle vector to be output

end








