% This function calcultes the weights for each particle based on the
% observation likelihood
%           S_bar(t)            4XM
%           outlier             1Xn
%           Psi(t)              1XnXM
% Outputs: 
%           S_bar(t)            4XM
function S_bar = weight(S_bar, Psi, outlier)

    % YOUR IMPLEMENTATION
    % Get M 
    M = size(S_bar,2);

    % Check if outliers
    if all(outlier) % All measurements are inliers; set weight to 1/M
        S_bar(4,:) = ones(1,M).*(1/M);
    else
        Psi_inliers = Psi(1,~outlier,:);
        % Compute unnormalized weights
        w_unnormalized = prod(Psi_inliers, 2);
        % Normalize weights
        S_bar(4,:) = w_unnormalized./sum(w_unnormalized);
    end
    
end