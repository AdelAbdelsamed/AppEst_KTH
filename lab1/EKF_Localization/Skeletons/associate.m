% This function performs the maximum likelihood association and outlier detection given a single measurement.
% Note that the bearing error lies in the interval [-pi,pi)
%           mu_bar(t)           3X1
%           sigma_bar(t)        3X3
%           z_i(t)              2X1
% Outputs: 
%           c(t)                1X1
%           outlier             1X1
%           nu^i(t)             2XN
%           S^i(t)              2X2XN
%           H^i(t)              2X3XN
function [c, outlier, nu, S, H] = associate(mu_bar, sigma_bar, z_i)

    % Import global variables
    global Q % measurement covariance matrix | 1X1
    global lambda_m % outlier detection threshold on mahalanobis distance | 1X1
    global map % map | 2Xn
    
    % YOUR IMPLEMENTATION %
    
    % Define no. of landmarks;
    N = size(map,2);

    % Define the matrices
    H = zeros(2,3,N);
    S = zeros(2,2,N);
    nu = zeros(2, N);
    m = zeros(1, N);
    ml_est = zeros(1,N);

    % Iterate over all landmarks in map
    for j = 1:N
        % Predict measurement
        z_pred_j = observation_model(mu_bar, j);

        % Compute the Jacobian of measurement
        H(:,:,j) = jacobian_observation_model(mu_bar, j, z_pred_j);
        
        % Compute scaling matrix S
        S(:,:,j) = H(:,:,j)*sigma_bar*H(:,:,j)' + Q;

        % Compute innovation
        nu(:,j) = z_i - z_pred_j;
        nu(2,j) = mod(nu(2,j) + pi, 2*pi) - pi; % ensure bearing is within the range [pi, pi]

        % Compute Mahalonobis distance
        m(1,j) = nu(:,j)'*inv(S(:,:,j))*nu(:,j);
        
        % Maximum likelihood
        ml_est(1,j) = 1/sqrt(det(2*pi*S(:,:,j)))*exp(-0.5*m(1,j)); 

    end

    % Compute the landmark with the feature that maximizes the likelihood
    [~, c] = max(ml_est);

    % See if outlier or not based on mahalanobis distance
    outlier = false;
    if m(1, c) > lambda_m
        outlier = true;
    end

end