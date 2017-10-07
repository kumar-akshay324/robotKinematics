%	AKSHAY KUMAR
%	ID - 842954269
%	QUESTION 2 - ASSIGNMENT 4 PART 1 
%	

%   i            d_i     theta_i     a_i_minus_1     alpha_i_minus_1
% 
%   1             0       theta_1         0               0         
%   2             0       theta_2         0              -90
%   3             d_3     theta_3         a_2             0 
%   4             d_4     theta_4         a_3            -90
%   5             0       theta_5         0               90
%   6             0       theta_6         0              -90

% The input to the function will be this 6x4 matrix DH_para where
% DH_para(4,4) would be angle alpha3 = -90
% This DH parameter table was derived in the previous question. 

% 0_to_6_T = 0_to_1_T * 1_to_2_T * 2_to_3_T * 3_to_4_T * 4_to_5_T * 5_to_6_T

function ee_pose = fk(DH_para)      % Function that calculates the final transformation matrix 0_to_6_T
                                    % It then calculates the position of
                                    % the end effector unit vector in
                                    % frame 6 with respect to frame 1

ee_pose = [0 0 0];                  % Initialising ee_pose row vector = [x_pos y_pos z_pos] 
final_trans_mat = eye(4,4);         % Initialising the matrix for the 0_to_6_T tranformation matrix
pos_mat = [0;0;0;1];                % Column matrix for the end effector's position EE = [px; py; pz; 1]

for i = 1:1:6
    
    tt= trans_mat_i(DH_para(i,1), DH_para(i,2), DH_para(i,3), DH_para(i,4));
    final_trans_mat = final_trans_mat * tt; 
    
end

ee_pose = final_trans_mat * pos_mat  % Gives the position coordinates of the end effector     
 
end 


