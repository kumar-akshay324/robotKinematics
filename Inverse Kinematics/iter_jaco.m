%   AKSHAY KUMAR
%   WPI ID - 842954269
%   EMAIL - akumar5@wpi.edu
%   SYNERGY OF HUMAN AND ROBOT - RBE595 - F17 - 191S
%   QUESTION 2 - ASSIGNMENT 4 - PART 2

% Implementation of Iterative Jacobian Pseudo Inverse Methodology on a 3
% DOF manipulator arm

% Figures and related derivation has been shown in the attached scanned pdf

function angles = iter_jaco(x_tar, y_tar, link_len)

t1 = pi/2;
t2 = pi/4;
t3 = pi/8;
l1 = link_len(1)
l2 = link_len(2)
l3 = link_len(3)

% com_trans_mat = fkt(0,0,0,t1) * fkt(0,0,l1,t2) * fkt(0,0,l2,t3);

% pos_cur = [x_cur y_cur];
t = [t1; t2; t3];
while true
     
%  for i = 1:5

x_cur  = l1*cos(t(1)) + l2*cos(t(1)+t(2)) + l3*cos(t(1)+t(2)+t(3))
y_cur  = l1*sin(t(1)) + l2*sin(t(1)+t(2)) + l3*sin(t(1)+t(2)+t(3)) 

Jac_mat = [-l1*sin(t(1))-l2*sin(t(1)+t(2))-l3*sin(t(1)+t(2)+t(3)) -l2*sin(t(1)+t(2))-l3*sin(t(1)+t(2)+t(3)) -l3*sin(t(1)+t(2)+t(3));
            l1*cos(t(1))+l2*cos(t(1)+t(2))+l3*cos(t(1)+t(2)+t(3))  l2*cos(t(1)+t(2))+l3*cos(t(1)+t(2)+t(3)) l3*cos(t(1)+t(2)+t(3)); 0 0 1];
   
x_del =  x_tar - x_cur;
y_del = y_tar - y_cur;
e_del =  sqrt((x_tar -x_cur)^2 + (y_tar - y_cur)^2)

d_del = [x_del; y_del; 0];

if (e_del <= 0.5)         % d_del is equivalent to the x_dot from the pseudo code 
     disp("Optimal Convergence Value Achieved")
     break;   
end

p = inv(Jac_mat);
t_dot = inv(Jac_mat) * d_del;
t = t + t_dot;

% if t(1)>2*pi
%     t(1) = t(1)-2*pi
% end
% if t(2)>2*pi
%     t(2) = t(2)-2*pi
% end
% if t(3)>2*pi
%     t(3) = t(3)-2*pi
% end    

end

x_cur  = l1*cos(t(1)) + l2*cos(t(1)+t(2)) + l3*cos(t(1)+t(2)+t(3))
y_cur  = l1*sin(t(1)) + l2*sin(t(1)+t(2)) + l3*sin(t(1)+t(2)+t(3))
angles = [t(1) t(2) t(3)]

end

    
    
    















