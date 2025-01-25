% This function performs the maximum likelihood association and outlier detection.
% Note that the bearing error lies in the interval [-pi,pi)
%           mu_bar(t)           3X1
%           sigma_bar(t)        3X3
%           z(t)                2Xn
% Outputs: 
%           c(t)                1Xn
%           outlier             1Xn
%           nu_bar(t)           2nX1
%           H_bar(t)            2nX3
function [c, outlier, nu_bar, H_bar] = batch_associate(mu_bar, sigma_bar, z)
        
        % YOUR IMPLEMENTATION %
        % Number of observations
        n = size(z, 2);

        % Define the matrices
        c = zeros(1, n);
        outlier = zeros(1, n);
        nu_bar = zeros(2*n, 1);
        H_bar = zeros(2*n, 3);
        
        % Iterate through the observations
        for i = 1:n
            % Associate observation
            % Get start and end index
            if i == 1
                s_ind = 1;
                e_ind = 2;
            else
                s_ind = 2*i-1;
                e_ind = 2*i;
            end
            [c(1,i), outlier(1,i), nu_bar_temp, ~, H_temp_bar] = associate(mu_bar, sigma_bar, z(:,i));

            H_bar(s_ind:e_ind,:) = H_temp_bar(:, :, c(1, i));
            nu_bar(s_ind:e_ind, 1) = nu_bar_temp(:, c(1,i));
        end

end