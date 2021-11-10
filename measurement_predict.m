function [yhat_last,K_k] = measurement_predict(xhat_k_last, H_k, P_k_last, R_k)
x = xhat_k_last(1); 
y = xhat_k_last(2); 
phi = xhat_k_last(3);
% rho = sqrt(x^2 + y^2); 

yhat_last = [x; y; phi]; 
S_k = H_k*P_k_last*H_k' + R_k; 
K_k = P_k_last*H_k'*inv(S_k); 

end

