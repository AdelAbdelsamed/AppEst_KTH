% This function is the implementation of the jacobian measurement model
% required for the update of the covariance function after incorporating
% the measurements
% Inputs:
%           x(t)        3X1
%           j           1X1 which landmark (map column)
%           z_j         2X1
% Outputs:  
%           H           2X3
function H = jacobian_observation_model(x, j, z_j)

    global map % map | 2Xn for n landmarks

    % YOUR IMPLEMENTATION %

    % Obtain the landmark
    landmark_j = map(:, j);
    
    % Define the Jacobian of h
    H = zeros(2, 3);

    H(1,1) = (x(1,1) - landmark_j(1, 1))/z_j(1, 1);
    H(1,2) = (x(2,1) - landmark_j(2, 1))/z_j(1, 1);
    H(2,1) = -(x(2,1) - landmark_j(2, 1))/(z_j(1, 1))^2;
    H(2,2) = (x(1,1) - landmark_j(1, 1))/(z_j(1, 1))^2;
    H(2,3) = -1.0;

end
