% This function performs the prediction step.
% Inputs:
%           S(t-1)            4XN
%           v                 1X1
%           omega             1X1
% Outputs:   
%           S_bar(t)          4XN
function [S_bar] = predict(S, v, omega, delta_t)

    % Comment out any S_bar(3, :) = mod(S_bar(3,:)+pi,2*pi) - pi before
    % running the test script as it will cause a conflict with the test
    % function. If your function passes, uncomment again for the
    % simulation.

    global R % covariance matrix of motion model | shape 3X3
    global M % number of particles
    
    % YOUR IMPLEMENTATION
    % Implementation 1:
    % Define the new set 
%     S_bar = zeros(4, M); 
%     % Compute the new particles
%     S_bar(1,:) = S(1,:) + delta_t.*v*cos(S(1,:));
%     S_bar(2,:) = S(2,:) + delta_t.*v*sin(S(2,:));
%     S_bar(3,:) = S(3,:) + delta_t.*omega;
%     S_bar(1:3,:) = S_bar(1:3,:) + mvnrnd(zeros(1, 3), R, M)'; 
    % Wrap theta around -pi to pi
    %S_bar(3, :) = mod(S_bar(3,:)+pi,2*pi) - pi;

    % Implementation 2:
    noise_std = sqrt([R(1,1); R(2,2); R(3,3)]);

    % Compute the new particles
    S_bar = S(1:3,:) + [delta_t .* v .* cos(S(1,:));
                        delta_t .* v .* sin(S(1,:));
                        delta_t .* omega .* ones(1, M)] + normrnd(0, repmat(noise_std, 1, M));

    % Wrap theta around -pi to pi
    %S_bar(3,:) = mod(S_bar(3,:) + pi, 2*pi) - pi;

    % Add weights back to the particle set
    S_bar = [S_bar; S(4,:)];

end