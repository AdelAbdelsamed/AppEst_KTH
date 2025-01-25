%% Function implementing the state equation of the DC-Motor 
% Inputs 
% x_k1  : State at time k-1                   nxN
% u     : Control at timr k                   1x1
% param : Struct containing model parameters
% Outputs
% x     : State at time k                     nxN
function x = dc_motor_state_transition(u, x_k1, param)
    
    % Get no. of points to perform the transition on
    N = size(x_k1, 2);
    % Define the new state vector
    x = zeros(3,N);

    % Compute current state
    x(1,:) = (1 - (param.R*param.Ts/param.L)).*x_k1(1,:) - (param.k_t*param.Ts/param.L).*x_k1(3,:)  + u*param.Ts/param.L;
    x(2,:) = x_k1(2,:) + param.Ts.*x_k1(3,:);
    x(3,:) = (param.k_t*param.Ts/param.J_es).*x_k1(1,:) + (1 - (param.d_m*param.Ts/param.J_es)).*x_k1(3,:) -sign(x_k1(3,:))*(param.Ts*param.T_c/param.J_es);

%     % Restrict angle between -pi and pi
%     x(2,:) = mod(x(2,:)+pi,2*pi)- pi;

end