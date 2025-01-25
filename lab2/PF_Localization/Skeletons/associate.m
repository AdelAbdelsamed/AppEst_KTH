% This function performs the ML data association
%           S_bar(t)                 4XM
%           z(t)                     2Xn
%           association_ground_truth 1Xn | ground truth landmark ID for
%           every measurement  
% Outputs: 
%           outlier                  1Xn    (1 means outlier, 0 means not outlier) 
%           Psi(t)                   1XnXM
%           c                        1xnxM
function [outlier, Psi, c] = associate(S_bar, z, association_ground_truth)
    if nargin < 3
        association_ground_truth = [];
    end
    global DATA_ASSOCIATION
    global landmark_ids
    global lambda_psi % threshold on average likelihood for outlier detection
    global Q % covariance matrix of the measurement model
    global M % number of particles
    global N % number of landmarks
    if size(DATA_ASSOCIATION,1) == 0
    	DATA_ASSOCIATION="on";
    end

    % YOUR IMPLEMENTATION
    % Determine the no. of observations
    n = size(z,2);

    % Define Variables 
    Psi = zeros(1,n,M);
    c = zeros(1,n,M);
    psi = zeros(n,N,M);
    z_km = zeros(N,2,M);
    
    % Define constants
    norm_const = 1/(2*pi*sqrt(det(Q)));
    Q_inv = inv(Q);

    % Compute all expected observations
    for k = 1:N
        z_km(k,:, :) = observation_model(S_bar, k);
    end

    for i = 1:n
        %z_i = repmat(z(:,i),1,M);
        for k = 1:N
        % Compute the innovation
        nu = repmat(z(:,i),1,M) - squeeze(z_km(k,:,:));
        % Wrap innovation between -pi and pi
        nu(2,:) = mod(nu(2,:)+pi,2*pi) - pi;
        % Compute Likelihood
        psi(i,k,:) = norm_const.*exp(-0.5 * sum((nu' * Q_inv) .* nu', 2));
        end
        % Arg Maximum over the landmarks
        [Psi(1,i,:), c(1,i,:)] = max(psi(i,:,:),[],2);
    end

    % Outlier detection
    outlier = mean(Psi, 3) <= lambda_psi;
 
end

        

