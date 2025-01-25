function valid = test_case_predict()

% load test data
load test_case_prediction.mat;

% store errors
errs_mu = ones(1,NUM_TEST);
errs_sigma = ones(1,NUM_TEST);

% import global variables
global R;

% iterate over test data
for i = 1 : NUM_TEST
    
    % get data for iteration from test data
    mu = MU(:,i);
    sigma = reshape(SIGMA(:,i),3,3);
    u = U(:,i);
    R = diag(RT(:,i));

    try
        [mu_bar, sigma_bar] = predict_(mu, sigma, u);
    catch exception
        break;
    end
    
    % store euclidean norm of difference vectors between ground truth
    % values and computed values
    errs_mu(i) = norm(MU_BAR(:,i) - mu_bar(:));
    errs_sigma(i) = norm(SIGMA_BAR(:,i) - sigma_bar(:));
end

% function valid if MSE over test data below threshold for each return
% value
mse_errs_mu = mean(errs_mu .^2);
mse_errs_sigma = mean(errs_sigma .^2);
THRESH_VALID = 1e-20;
valid_mu = mse_errs_mu < THRESH_VALID;
valid_sigma = mse_errs_sigma < THRESH_VALID;

if ~valid_mu
    fprintf('the mu calculated in predict seems to be incorrect, mse=%f\n', mse_errs_mu);
end

if ~valid_sigma
    fprintf('the sigma calculated in predict seems to be incorrect, mse=%f\n', mse_errs_sigma);
end
valid = valid_mu && valid_sigma;

end