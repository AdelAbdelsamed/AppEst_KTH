% This function calculates the odometry information.
% Inputs:
%           e_L(t):         1X1
%           e_R(t):         1X1
%           E_T:            1X1
%           B:              1X1
%           R_L:            1X1
%           R_R:            1X1
%           delta_t:        1X1
%           mu(t-1):        3X1
% Outputs:
%           u(t):           3X1
function u = calculate_odometry(e_R, e_L, E_T, B, R_L, R_R, delta_t, mu)

    if ~delta_t
        u = [0;0;0];
        return;
    end

    % YOUR IMPLEMENTATION %
    
    % Compute angular velocity of right wheel 
    omega_R = 2*pi*e_R/(delta_t*E_T);
    % Compute angular velocity of left wheel
    omega_L = 2*pi*e_L/(delta_t*E_T);

    % Translational and Angular velocity of the robot
    omega = (omega_R*R_R - omega_L*R_L)/B;
    velocity = (omega_R*R_R + omega_L*R_L)/2;

    % Compute u
    u(1,1) = velocity*delta_t*cos(mu(3,1));
    u(2,1) = velocity*delta_t*sin(mu(3,1));
    u(3,1) = omega*delta_t;

end