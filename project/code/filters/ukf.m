% This function performs one iteration of UKF localization and follows the
% algorithm in Table 3.4 Page 70 in Probabilistic Robotics Book by Thrun et al.
% Inputs:
%           mu(t-1)                   3X1
%           sigma(t-1)                3X3
%           u                         1X1
%           z                         1Xn
%           g                         Function handle for state transition fcn
%           h                         Function handle for measurement fcn
% Outputs:
%           mu(t)                     3X1
%           sigma(t)                  3X3
function [mu, sigma] = ukf(mu_old, sigma_old, u, z, g, h, ukf_param, model_param)

    n = length(mu_old);   % Dimension of the state
    lambda = ukf_param.alpha^2 * (n + ukf_param.kappa) - n;
    gamma = sqrt(n + lambda);

    % Compute weights
    Wm = [lambda / (n + lambda), repmat(1 / (2 * (n + lambda)), 1, 2 * n)];
    Wc = Wm;
    Wc(1) = Wc(1) + (1 - ukf_param.alpha^2 + ukf_param.beta);

    % Generate sigma points
    sqrtSigma = chol(sigma_old, 'lower');
    sigmaPoints = [mu_old, mu_old + gamma * sqrtSigma, mu_old - gamma * sqrtSigma];
    sigmaPoints(2,:) = mod(sigmaPoints(2,:)+pi,2*pi)- pi;

    % Predict sigma points
    X_t_star = g(u, sigmaPoints, model_param);
    X_t_star(2,:) = mod(X_t_star(2,:)+pi,2*pi)- pi;

    % Predicted mean and covariance
    mu_t_bar = X_t_star * Wm';
    Sigma_t_bar = ukf_param.R;
    for i = 1:2 * n + 1
        diff = X_t_star(:, i) - mu_t_bar;
        Sigma_t_bar = Sigma_t_bar + Wc(i) * (diff * diff');
    end

    % Transform sigma points through observation function
    Z_t_star = h(X_t_star);

    % Predicted observation mean and covariance
    z_t_bar = Z_t_star * Wm';
    S_t = ukf_param.Q;
    for i = 1:2 * n + 1
        diff = Z_t_star(:, i) - z_t_bar;
        S_t = S_t + Wc(i) * (diff * diff');
    end

    % Cross-covariance
    Sigma_xz = zeros(n, size(z, 1));
    for i = 1:2 * n + 1
        Sigma_xz = Sigma_xz + Wc(i) * ((X_t_star(:, i) - mu_t_bar) * (Z_t_star(:, i) - z_t_bar)');
    end

    % Kalman gain
    K_t = Sigma_xz / S_t;

    % Update state mean and covariance
    nu = mod(z - z_t_bar +pi,2*pi) -pi;
    mu = mu_t_bar + K_t * nu;
    mu(2,:) = mod(mu(2,:)+pi,2*pi)- pi; % Angle between [-pi, pi]
    sigma = Sigma_t_bar - K_t * S_t * K_t';
end
