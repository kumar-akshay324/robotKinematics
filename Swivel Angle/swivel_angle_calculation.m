% AKSHAY KUMAR 
% WPI ID - 842954269  
% Email - akumar5@wpi.edu
% SYNERGY OF HUMAN AND ROBOT - RBE 595 - F17 - 191S
% ASSIGNMENT 5
%
% Coordinates of the points Pw, Ps and Pe are given with temporal
% distribution
%
% We have to compute the swivel angle as temporal distribution
%

n_vector = zeros(167,3);
len_n_vector = zeros(167,1);

f_vector = zeros(167,3);
len_f_vector = zeros(167,1);

f_dash_vector = zeros(167,1);
u_vector = zeros(167,3);

a_vector_x = 0;
a_vector_y = 0;
a_vector_z = -1;

cos_theta = zeros(167,1);
sin_theta = zeros(167,1);

theta = zeros(167,1);

syms x_cap y_cap z_cap

f_dash_lenn = zeros(167,1);
u_vector_lenn = zeros(167,1);

for i = 1:167
    
    % Computation  of n vector
    
    len_n_vector(i,1) = sqrt((Pw(i,1) - Ps(i,1))^2 + (Pw(i,2) - Ps(i,2))^2 + (Pw(i,3) - Ps(i,3))^2);  % LENGTH OF THE Pw_Ps_vector
        
    n_vector(i,1) = (Pw(i,1) - Ps(i,1))/len_n_vector(i,1);      % Calculation of the x, y and z          x component
    n_vector(i,2) = (Pw(i,2) - Ps(i,2))/len_n_vector(i,1);      % components of the Pw_Ps vector         y component
    n_vector(i,3) = (Pw(i,3) - Ps(i,3))/len_n_vector(i,1);      %                                        z component
        

    n_vector_final = n_vector(i,1)*x_cap + n_vector(i,2)*y_cap + n_vector(i,3)*z_cap;     % This is the n unit vector in the direction of Pe-Ps
   
    %------------------------------------------------------------------------------------------------------------------------------------ 
    
    % Computation of f vector
    
    len_f_vector(i,1) = sqrt((Pe(i,1) - Ps(i,1))^2 + (Pe(i,2) - Ps(i,2))^2 + (Pe(i,3) - Ps(i,3))^2);  % LENGTH OF THE Pe_Ps_vector
    
    f_vector(i,1) = (Pe(i,1) - Ps(i,1))/len_f_vector(i,1);      % Calculation of the x, y and z          x component
    f_vector(i,2) = (Pe(i,2) - Ps(i,2))/len_f_vector(i,1);      % components of the Pe_Ps vector         y component
    f_vector(i,3) = (Pe(i,3) - Ps(i,3))/len_f_vector(i,1);      %                                        z component
    
    f_vector_final = f_vector(i,1)*x_cap + f_vector(i,2)*y_cap + f_vector(i,3)*z_cap;     % This is the f unit vector in the direction of Pe-Ps

    %------------------------------------------------------------------------------------------------------------------------------------
    
    % Computation of f_dash vector
    
    f_dot_n = f_vector(i,1)*n_vector(i,1) + f_vector(i,2)*n_vector(i,2) + f_vector(i,3)*n_vector(i,3);  % Dot product of f_vector and n_vector
    
    len_f_dash_vector = sqrt((f_vector(i,1) - (f_dot_n)* n_vector(i,1))^2 + (f_vector(i,2) - (f_dot_n)* n_vector(i,2))^2 + (f_vector(i,3) - (f_dot_n)* n_vector(i,3))^2);
    
    f_dash_vector(i,1) = (f_vector(i,1) - (f_dot_n)* n_vector(i,1))/len_f_dash_vector;
    f_dash_vector(i,2) = (f_vector(i,2) - (f_dot_n)* n_vector(i,2))/len_f_dash_vector;
    f_dash_vector(i,3) = (f_vector(i,3) - (f_dot_n)* n_vector(i,3))/len_f_dash_vector;

    f_dash_vector_final = f_dash_vector(i,1)*x_cap + f_dash_vector(i,2)*y_cap + f_dash_vector(i,3)*z_cap;
    
    f_dash_lenn = sqrt(f_dash_vector(i,1)^2 + f_dash_vector(i,2)^2 + f_dash_vector(i,3)^2);     % Final length of the f_dash vector
    
    %------------------------------------------------------------------------------------------------------------------------------------
    
    % Computation of u vector
    
    a_dot_n = n_vector(i,1) * a_vector_x + n_vector(i,2) * a_vector_y + n_vector(i,3) * a_vector_z;     % Dot product of a_vector and n_vector
    
    len_u_vector = sqrt((a_vector_x - a_dot_n * n_vector(i,1))^2 + (a_vector_y - a_dot_n * n_vector(i,2))^2 + (a_vector_z - a_dot_n * n_vector(i,3))^2); 
    
    u_vector(i,1) = (a_vector_x - a_dot_n * n_vector(i,1))/len_u_vector;
    u_vector(i,2) = (a_vector_y - a_dot_n * n_vector(i,2))/len_u_vector; 
    u_vector(i,3) = (a_vector_z - a_dot_n * n_vector(i,3))/len_u_vector; 
    
    u_vector_final = u_vector(i,1) * x_cap + u_vector(i,2) * y_cap + u_vector(i,3) * z_cap;

    u_vector_lenn = sqrt(u_vector(i,1)^2 + u_vector(i,2)^2 + u_vector(i,3)^2);      % Final length of the u_vector
    
    %-----------------------------------------------------------------------------------------------------------------------------------
    
    % Computation of cos_theta
    
    f_dash_dot_u  = (f_dash_vector(i,1)*u_vector(i,1) + f_dash_vector(i,2)*u_vector(i,2) + f_dash_vector(i,3)*u_vector(i,3)); % Dot product of f_dash 
                                                                                                                              % and u_vector for calcultion of cos theta  
    cos_theta(i,1) = (f_dash_dot_u)/(f_dash_lenn* u_vector_lenn);           % cos theta for swivel angle theta
    
    % Computation of sin_theta
    
    f_dash_cross_u_x = f_dash_vector(i,2) * u_vector(i,3) - f_dash_vector(i,3) * u_vector(i,2);         % Cross product of the f_dash vector 
    f_dash_cross_u_y = -(f_dash_vector(i,1) * u_vector(i,3) - f_dash_vector(i,3) * u_vector(i,1));      % and the u vector
    f_dash_cross_u_z = f_dash_vector(i,1) * u_vector(i,2) - f_dash_vector(i,2) * u_vector(i,1);         

    sin_theta(i,1) = ((f_dash_cross_u_x*n_vector(i,1)) + (f_dash_cross_u_y*n_vector(i,2)) + (f_dash_cross_u_z*n_vector(i,3)) )/(f_dash_lenn * u_vector_lenn);
    % Calculation of sin theta for swivel angle theta
    
    theta(i,1) = atan2d(sin_theta(i,1),cos_theta(i,1));			% Calculation  of the swivel angle theta in degrees
    
end

x_points = 1:1:167;
plot(x_points, theta, 'g--*')
xlabel('Intervals');
ylabel('Value of Swivel angle theta(in degrees)');
title('Swivel angle distribution');

    
    
    
