% This function performs one iteration of KF localization and follows the
% algorithm in Table 3.4 Page 59 in Probabilistic Robotics Book by Thrun et al.
% Inputs:
%           mu(t-1)                   3X1
%           sigma(t-1)                3X3
%           u                         1X1
%           z                         1Xn
%           ekf_param                 struct containing the tuning parameters
%           model_param               struct containing the model parameters
% Outputs:
%           mu(t)                     3X1
%           sigma(t)                  3X3
function [mu, sigma] = ekf(mu_old, sigma_old, u, z, ekf_param, model_param)

    % State variable dimension
    n = length(mu_old);
    % Predict step
    mu_bar    = dc_motor_approx_state_transition(u, mu_old, model_param);
    mu_bar(2,:) = mod(mu_bar(2,:)+pi,2*pi)- pi; % Restrict angle between -pi and pi
    Jacobian_G = G(mu_bar, model_param); % Compute Jacobian at mu(t-1)
    sigma_bar = Jacobian_G*sigma_old*Jacobian_G' + ekf_param.R;
    
    % Update step
    K = sigma_bar*ekf_param.H'/(ekf_param.H*sigma_bar*ekf_param.H' + ekf_param.Q); % Kalman Gain
    nu = mod(z- ekf_param.H*mu_bar + pi,2*pi)- pi; % Restrict angle between -pi and pi
    mu = mu_bar + K*nu;
    mu(2,:) = mod(mu(2,:)+pi,2*pi)- pi; % Restrict angle between -pi and pi
    sigma = (eye(n) - K*ekf_param.H)*sigma_bar;

end


function x = dc_motor_approx_state_transition(u, x_k1, param)
    % Get no. of points to perform the transition on
    N = size(x_k1, 2);
    % Define the new state vector
    x = zeros(3,N);

    % Compute current state
    x(1,:) = (1 - (param.R*param.Ts/param.L)).*x_k1(1,:) - (param.k_t*param.Ts/param.L).*x_k1(3,:)  + u*param.Ts/param.L;
    x(2,:) = x_k1(2,:) + param.Ts.*x_k1(3,:);
    x(3,:) = (param.k_t*param.Ts/param.J_es).*x_k1(1,:) + (1 - (param.d_m*param.Ts/param.J_es)).*x_k1(3,:) -(2/pi)*atan(param.p*x_k1(3,:))*(param.Ts*param.T_c/param.J_es);

end


function G = G(x, param)
    G = [1-(param.R/param.L)*param.Ts  , 0, -param.k_t*param.Ts/param.L; 
        0                              , 1, param.Ts                  ;
        (param.k_t*param.Ts/param.J_es), 0, ...
        1-(param.d_m*param.Ts/param.J_es)-(2*param.Ts*param.T_c/(pi*param.J_es))*(param.p/(1+(param.p*x(3))^2))];

end