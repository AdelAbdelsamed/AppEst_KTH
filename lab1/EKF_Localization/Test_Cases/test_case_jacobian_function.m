function valid =test_case_jacobian_function()

% load test data
load test_case_jac.mat;

% import global variables
global map;
map = M;

% store errors
errs = ones(1,NUM_TEST);

% iterate over test data
for i = 1 : NUM_TEST
    try
        TH = jacobian_observation_model(X(:,i), J(i), Z(:,i));
    catch exception
        break;
    end
       
    % store euclidean norm of difference vector between ground truth
    % jacobian H and computed jacobian TH
    errs(i) = norm(TH - H(i*2-1:i*2,:));
end

% function valid if MSE over test data below threshold
mse_err = mean(errs .^2);
THRESH_VALID = 1e-20;
valid = mse_err < THRESH_VALID;
   
end