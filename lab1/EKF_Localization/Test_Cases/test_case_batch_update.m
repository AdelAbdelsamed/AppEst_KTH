function valid = test_case_batch_update()

% load test data
load test_case_batch_update.mat;

% store errors
errs_mu = ones(1,NUM_TEST);
errs_sigma = ones(1,NUM_TEST);

% iterate over test data
for i = 1 : NUM_TEST
    
    % get data for iteration from test data
    mu_bar = MU_BAR(:,i);
    sigma_bar = reshape(SIGMA_BAR(:,i),3,3);
    valid_ixs = find(~OUTLIER(:,i)); % the indices of inliers
    ix = [2*(valid_ixs-1)+1;2*(valid_ixs-1)+2];
    ix = ix(:);
    nu_bar = NU_BAR(:,i);
    nu_bar = nu_bar(ix);
    H_bar = reshape(H_BAR(:,i),2*NUM_OBS,3);
    H_bar = H_bar(ix,:);
    n = length(valid_ixs);
    Q_bar = zeros(2*n,2*n);
    q = diag(Q(:,i));
    
    for j=1:n
        ii= 2*j + (-1:0);
        Q_bar(ii,ii) = q;
    end
    
    try
        [mu,sigma] = batch_update(mu_bar,sigma_bar,H_bar,Q_bar,nu_bar);
    catch exception
        break;
    end
          
    % store euclidean norm of difference vectors between ground truth
    % values and computed values
    errs_mu(i) = norm(MU(:,i) - mu);
    errs_sigma(i) = norm(SIGMA(:,i) - sigma(:));
end

% function valid if MSE over test data below threshold for each return
% value
mse_errs_mu = mean(errs_mu .^2);
mse_errs_sigma = mean(errs_sigma .^2);

THRESH_VALID = 1e-20;

valid_mu = mse_errs_mu < THRESH_VALID;
valid_sigma = mse_errs_sigma < THRESH_VALID;

valid = valid_mu && valid_sigma;

if ~valid_mu
    fprintf('mu computed in batch_update.m seems to be wrong, mse=%f\n', mse_errs_mu);
end

if ~valid_sigma
    fprintf('sigma computed in batch_update.m seems to be wrong, mse=%f\n', mse_errs_sigma);
end

end