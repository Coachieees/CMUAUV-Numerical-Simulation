% Time
t = out.eta_sim.time;

% Reshape data
eta = squeeze(out.eta_sim.signals.values)';     
nu = squeeze(out.nu_sim.signals.values)';       
thrust = squeeze(out.thrust_sim.signals.values)';

% ===== Create Tables =====

T_eta = array2table([t eta], 'VariableNames', ...
    {'Time','x','y','z','phi','theta','psi'});

T_nu = array2table([t nu], 'VariableNames', ...
    {'Time','u','v','w','p','q','r'});

T_thrust = array2table([t thrust], 'VariableNames', ...
    {'Time','T1','T2','T3','T4','T5','T6','T7','T8'});

% ===== Write to Excel (same file, different sheets) =====

filename = 'ROV_simulation_output.xlsx';

writetable(T_eta, filename, 'Sheet', 'eta');
writetable(T_nu, filename, 'Sheet', 'nu');
writetable(T_thrust, filename, 'Sheet', 'thrust');
