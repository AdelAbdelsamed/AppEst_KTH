function valid = test_case_observation_model()

% load test data
load test_case_obs.mat;

% store errors
errs = ones(1,NUM_TEST);

% import global variables
global map;
map = M;

% iterate over test data
for i = 1 : NUM_TEST
    try
        tz = observation_model(X(:, i), J(i));
    catch exception
        break;
    end
    
    % store euclidean norm of difference vector between ground truth
    % observation z and compute observation tz
    errs(i) = norm(tz - Z(:,i));
end   

% function valid if MSE over test data below threshold
mse_err = mean(errs.^2);
THRESH_VALID = 1e-20;
valid = mse_err < THRESH_VALID;

end