% This function performs one iteration of KF localization and follows the
% algorithm in Table 3.1 Page 34 in Probabilistic Robotics Book by Thrun et al.
% Inputs:
%           mu(t-1)                   3X1
%           sigma(t-1)                3X3
%           u                         1X1
%           z                         1Xn
%           kf_param                  struct containing the tuning parameters
%           model_param               struct containing the model parameters
% Outputs:
%           mu(t)                     3X1
%           sigma(t)                  3X3
function [mu, sigma] = kf(mu_old, sigma_old, u, z, kf_param)

    % State variable dimension
    n = length(mu_old);
    % Predict step
    mu_bar    = kf_param.A*mu_old + kf_param.B*u;
    mu_bar(2,:) = mod(mu_bar(2,:)+pi,2*pi)- pi; % Restrict angle between -pi and pi
    sigma_bar = kf_param.A*sigma_old*kf_param.A' + kf_param.R;
    
    % Update step
    K = sigma_bar*kf_param.C'/(kf_param.C*sigma_bar*kf_param.C' + kf_param.Q); % Kalman Gain
    nu = mod(z- kf_param.C*mu_bar + pi,2*pi)- pi; % Restrict angle between -pi and pi
    mu = mu_bar + K*nu;
    mu(2,:) = mod(mu(2,:)+pi,2*pi)- pi; % Restrict angle between -pi and pi
    sigma = (eye(n) - K*kf_param.C)*sigma_bar;

end