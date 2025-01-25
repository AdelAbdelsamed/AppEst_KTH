% This function performs the prediction step.
% Inputs:
%           mu(t-1)           3X1   
%           sigma(t-1)        3X3
%           u(t)              3X1
% Outputs:   
%           mu_bar(t)         3X1
%           sigma_bar(t)      3X3
function [mu_bar, sigma_bar] = predict_(mu, sigma, u)
    
    global R % covariance matrix of motion model | shape 3X3

    % YOUR IMPLEMENTATION %

    % Compute the mean vector
    mu_bar = mu + u; 

    % Compute Jacobian of g(u_t,x_t-1)
    G = eye(3);
    G(1,3) = -u(2,1);
    G(2,3) = u(1,1);

    % Compute the covariance matrix
    sigma_bar = G*sigma*G' + R;

end
