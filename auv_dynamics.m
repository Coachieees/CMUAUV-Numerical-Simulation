function nu_dot = auv_dynamics(eta, nu, tau, M, M_inv, Dl, Dq, W, B, rg)
    % Extract states as explicit scalars to prevent dimension mismatch
    u = nu(1); v = nu(2); w = nu(3);
    p = nu(4); q = nu(5); r = nu(6);
    
    phi = eta(4); theta = eta(5); psi = eta(6);
    
    % Ensure rg, W, and B are treated as the correct dimensions
    xg = rg(1); yg = rg(2); zg = rg(3);
    W_val = W(1); % Force to scalar
    B_val = B(1); % Force to scalar
    
    % 1. Restoring Forces g(eta) - Built as a direct column vector
    g = [ (W_val - B_val) * sin(theta);
         -(W_val - B_val) * cos(theta) * sin(phi);
         -(W_val - B_val) * cos(theta) * cos(phi);
          zg * W_val * cos(theta) * sin(phi) - yg * W_val * cos(theta) * cos(phi);
          zg * W_val * sin(theta) + xg * W_val * cos(theta) * cos(phi);
         -xg * W_val * cos(theta) * sin(phi) - yg * W_val * sin(theta) ];
    
    % 2. Damping Matrix D(nu)
    % diag(abs(nu)) creates a 6x6 matrix
    D = Dl + Dq * diag(abs(nu));
    
    % 3. Coriolis Matrix C(nu)
    M11 = M(1:3, 1:3); M12 = M(1:3, 4:6);
    M21 = M(4:6, 1:3); M22 = M(4:6, 4:6);
    
    nu1 = nu(1:3); nu2 = nu(4:6);
    
    v1 = M11*nu1 + M12*nu2;
    v2 = M21*nu1 + M22*nu2;
    
    C = zeros(6,6);
    C(1:3, 4:6) = -Smtrx(v1);
    C(4:6, 1:3) = -Smtrx(v1);
    C(4:6, 4:6) = -Smtrx(v2);
    
    % 4. Calculate Accelerations
    nu_dot = M_inv * (tau - C*nu - D*nu - g);
end

% Helper function for Cross Product Skew-Symmetric Matrix
function S = Smtrx(a)
    S = [     0, -a(3),  a(2);
           a(3),     0, -a(1);
          -a(2),  a(1),     0 ];
end
