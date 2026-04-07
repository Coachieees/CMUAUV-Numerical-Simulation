function X_ref = circular_trajectory(t)
    R = 1.0;            % Radius of the circle (m)
    Z_target = 1.0;     % Target depth (m)
    omega = 0.1;        % Angular velocity in rad/s
    T_dive = 20.0;      % Time allowed to dive and stabilize (s)
    
    if t < T_dive
        x = 0;
        y = 0;
        z = Z_target;
        
        phi = 0;
        theta = 0;
        psi = 0;
        
        u = 0; v = 0; w = 0;
        p = 0; q = 0; r_yaw = 0;
        
    else
        t_circ = t - T_dive; 
        
        x = R * sin(omega * t_circ);
        y = R * (1 - cos(omega * t_circ)); 
        z = Z_target;
        
        phi = 0;
        theta = 0;
        psi = omega * t_circ; 
        
        u = R * omega; 
        v = 0;
        w = 0;
        p = 0;
        q = 0;
        r_yaw = omega; 
    end
    
    X_ref = [u; v; w; p; q; r_yaw; x; y; z; phi; theta; psi];
end
