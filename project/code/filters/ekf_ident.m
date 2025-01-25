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
function [mu, sigma] = ekf_ident(mu_old, sigma_old, u, z, ekf_param, model_param)

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
    % Dynamic States
    x(1,:) = (1 - (param.R*param.Ts/param.L)).*x_k1(1,:) - (param.k_t*param.Ts/param.L).*x_k1(3,:)  + u*param.Ts/param.L;
    x(2,:) = x_k1(2,:) + param.Ts.*x_k1(3,:);
    x(3,:) = (param.k_t*param.Ts./x_k1(5,:)).*x_k1(1,:) + (1 - (x_k1(4,:).*param.Ts./x_k1(5,:))).*x_k1(3,:) -(2/pi)*atan(param.p*x_k1(3,:))*(param.Ts.*x_k1(6,:)./x_k1(5,:));
    % Static states
    x(4,:) = x_k1(4,:);
    x(5,:) = x_k1(5,:);
    x(6,:) = x_k1(6,:);
end


function G = G(x, param)
    temp = -(x(1)*param.k_t*param.Ts)/x(5)^2 + (x(3)*x(4)*param.Ts)/x(5)^2 + (2*param.Ts*x(6)/pi)*atan(param.p*x(3))/x(5)^2;
    G = [1-(param.R/param.L)*param.Ts  , 0, -param.k_t*param.Ts/param.L, 0, 0, 0; 
        0                              , 1, param.Ts, 0, 0, 0                   ;
        (param.k_t*param.Ts/x(5)), 0, ...
        1-(x(4)*param.Ts/x(5))-(2*param.Ts*x(6)/(pi*x(5)))*(param.p/(1+(param.p*x(3))^2)), -param.Ts*(x(3)/x(5)), temp, (-2*param.Ts/pi)*atan(param.p*x(3))/x(5);
        0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 1, 0
        0, 0, 0, 0, 0, 1];

end