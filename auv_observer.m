function X_hat_dot = auv_observer(X_hat, Y, U, A_sys, B_sys, C, L)
    % Calculate simulated sensor output
    Y_hat = C * X_hat;

    % Calculate state derivative with correction
    X_hat_dot = A_sys * X_hat + B_sys * U + L * (Y - Y_hat);
end
