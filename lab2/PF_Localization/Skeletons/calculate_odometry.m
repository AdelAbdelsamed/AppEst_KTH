% This function calculates the odometry information
% Inputs:
%           e_L(t):         1X1
%           e_R(t):         1X1
%           E_T:            1X1
%           B:              1X1
%           R_L:            1X1
%           R_R:            1X1
%           delta_t:        1X1
% Outputs:
%           v:              1X1
%           omega:          1X1
function [v, omega] = calculate_odometry(e_R, e_L, E_T, B, R_R, R_L, delta_t)

    if ~delta_t
        v = 0;
        omega = 0;
        return;
    end
    
    % YOUR IMPLEMENTATION

    % Compute angular velocity of right wheel 
    omega_R = 2*pi*e_R/(delta_t*E_T);
    % Compute angular velocity of left wheel
    omega_L = 2*pi*e_L/(delta_t*E_T);

    % Translational and Angular velocity of the robot
    omega = (omega_R*R_R - omega_L*R_L)/B;
    v = (omega_R*R_R + omega_L*R_L)/2;
   
end
