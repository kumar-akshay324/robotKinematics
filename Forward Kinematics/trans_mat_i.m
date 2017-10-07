%	AKSHAY KUMAR
%	ID - 842954269
%	QUESTION 2 - ASSIGNMENT 4 PART 1
% Compute the transformation matrix for transformation from frame i-1 to 
% frame i

function trans_mat = trans_mat_i(d_i, theta_i, a_i_minus_1, alpha_i_minus_1)

% Matrix for twist angle alpha(i-1) between z(i-1) and z(i) axes around x(i-1) 
R_X_alpha_i_minus_1 = [1 0 0 0; 0 cosd(alpha_i_minus_1) -sind(alpha_i_minus_1) 0; 0 sind(alpha_i_minus_1) cosd(alpha_i_minus_1) 0; 0 0 0 1];

% Matric for link length a(i-1) between z(i-1) and z(i) axes along x(i-1)
D_X_a_i_minus_1 = [1 0 0 a_i_minus_1; 0 1 0 0; 0 0 1 0; 0 0 0 1];

% Matrix for joint angle theta_i between x(i-1) and x(i) axes around z(i)
R_Z_theta_i = [cosd(theta_i) -sind(theta_i) 0 0; sind(theta_i) cosd(theta_i) 0 0; 0 0 1 0; 0 0 0 1];

% Matrix for displacement d-i between x(i-1) and x(i) axes along z(i) 
D_Z_d_i = [1 0 0 0; 0 1 0 0; 0 0 1 d_i; 0 0 0 1];

% Transformation matrix for transformation between frame i-1 and frame i
trans_mat = R_X_alpha_i_minus_1 * D_X_a_i_minus_1 * R_Z_theta_i * D_Z_d_i; 

end
