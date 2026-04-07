function eta_dot = auv_kinematics(eta, nu)
    phi = eta(4);
    theta = eta(5);
    psi = eta(6);
    
    % Linear velocity transformation J1
    J1 = [cos(psi)*cos(theta), -sin(psi)*cos(phi) + cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi) + cos(psi)*cos(phi)*sin(theta);
          sin(psi)*cos(theta),  cos(psi)*cos(phi) + sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi) + sin(theta)*cos(phi)*sin(psi);
          -sin(theta),          cos(theta)*sin(phi),                               cos(theta)*cos(phi)];
          
    % Angular velocity transformation J2
    J2 = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
          0, cos(phi),           -sin(phi);
          0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
          
    J = zeros(6,6);
    J(1:3, 1:3) = J1;
    J(4:6, 4:6) = J2;
    
    eta_dot = J * nu;
end
