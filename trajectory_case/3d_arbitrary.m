function X_ref = arbitrary_trajectory(t)
    Rx = 1.0;            % X-axis amplitude (+/- 1m)
    Ry = 1.0;            % Y-axis amplitude (+/- 1m)
    Z_center = 1.5;      % Z-axis center depth (1.5m)
    Z_amp = 0.5;         % Z-axis amplitude (+/- 0.5m gives 1m to 2m)
    omega = 0.05;        % Speed of the trajectory
    T_dive = 15.0;       % Time allowed to dive and stabilize (seconds)
    
    initial_yaw = atan2(2 * Ry * omega, Rx * omega);

    if t < T_dive
        x = 0;
        y = 0;
        z = 1.5;
        
        phi = 0;
        theta = 0;
        psi = initial_yaw;
        
        u = 0; v = 0; w = 0;
        p = 0; q = 0; r_yaw = 0;
        
    else
        % ---------------------------------------------------
        % PHASE 2: 3D Infinity Path (Lissajous Curve)
        % ---------------------------------------------------
        t_path = t - T_dive; 
        
        % 1. Position Equations
        x = Rx * sin(omega * t_path);
        y = Ry * sin(2 * omega * t_path); % 2*omega makes it cross over itself (Infinity shape)
        z = Z_center - Z_amp * cos(omega * t_path); % Sweeps between 1.0m and 2.0m
        
        % 2. Earth-Fixed Velocities (Mathematical Derivatives)
        x_dot = Rx * omega * cos(omega * t_path);
        y_dot = 2 * Ry * omega * cos(2 * omega * t_path);
        z_dot = Z_amp * omega * sin(omega * t_path);
        
        % 3. Earth-Fixed Accelerations (Needed for Yaw Rate)
        x_ddot = -Rx * omega^2 * sin(omega * t_path);
        y_ddot = -4 * Ry * omega^2 * sin(2 * omega * t_path);
        
        % 4. Target Orientations
        phi = 0;
        theta = 0;
        psi = atan2(y_dot, x_dot); % Always point tangent to the path
        
        % 5. Body-Fixed Target Velocities
        % Forward speed is the combined horizontal speed
        u = sqrt(x_dot^2 + y_dot^2); 
        v = 0;     % No sideways slipping requested
        w = z_dot; % Plunge speed
        
        % 6. Target Angular Rates
        p = 0;
        q = 0;
        % Dynamic yaw rate formula based on the curvature of the path
        r_yaw = (x_dot * y_ddot - y_dot * x_ddot) / (x_dot^2 + y_dot^2); 
    end
    
    % Construct the 12x1 reference state vector
    X_ref = [u; v; w; p; q; r_yaw; x; y; z; phi; theta; psi];
end
