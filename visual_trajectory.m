%% AUV Interactive Real-Time 3D & 2D Trajectory Visualization
% Extract actual AUV data
time = out.eta_sim.time;
eta = squeeze(out.eta_sim.signals.values)'; 
nu = squeeze(out.nu_sim.signals.values)'; 
x = eta(:, 1); y = eta(:, 2); z = eta(:, 3);
phi = eta(:, 4); theta = eta(:, 5); psi = eta(:, 6);
% RENAMED to avoid conflict with init_auv 'v' (thruster directions)
vel_u = nu(:, 1); vel_v = nu(:, 2); vel_w = nu(:, 3);

% Extract Reference Target data (Assuming X_ref_data is 12x1 states)
ref_signals = squeeze(out.X_ref_data.signals.values)';
x_ref = ref_signals(:, 7); 
y_ref = ref_signals(:, 8); 
z_ref = ref_signals(:, 9);

% Calculate global limits with margin so graphs don't jump around
margin = 1.0;
xlims = [min([x; x_ref])-margin, max([x; x_ref])+margin];
ylims = [min([y; y_ref])-margin, max([y; y_ref])+margin];
zlims = [min([z; z_ref])-margin, max([z; z_ref])+margin];

% Check if the reference trajectory is a static point
is_static = (max(x_ref)-min(x_ref) < 1e-4) && (max(y_ref)-min(y_ref) < 1e-4) && (max(z_ref)-min(z_ref) < 1e-4);

%% 1. Figure & UI Setup
fig = figure('Name', 'AUV Trajectory Analysis', 'Color', 'w', 'Position', [100, 50, 1200, 800]);
% State 0 = Waiting to start, 1 = Running, 2 = Finished
setappdata(fig, 'runState', 0); 

% Create Speed Label
uicontrol('Parent', fig, 'Style', 'text', ...
    'String', 'Playback:', ...
    'Units', 'normalized', ...
    'Position', [0.15, 0.015, 0.06, 0.04], ...
    'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', 'w');

% Create Speed Dropdown Menu
speed_options = {'1x Speed', '2x Speed', '4x Speed', '10x Speed'};
speed_factors = [1, 2, 4, 10];
setappdata(fig, 'speedFactors', speed_factors);

speedMenu = uicontrol('Parent', fig, 'Style', 'popupmenu', ...
    'String', speed_options, ...
    'Units', 'normalized', ...
    'Position', [0.22, 0.02, 0.10, 0.05], ...  
    'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', 'w');
setappdata(fig, 'speedMenu', speedMenu);

% Create the Start/Restart Button
playBtn = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
    'String', 'Start Simulation', ...
    'Units', 'normalized', ...
    'Position', [0.35, 0.02, 0.12, 0.05], ...  
    'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.2 0.8 0.2], 'ForegroundColor', 'w', ...
    'Callback', @playButtonCallback);

% Create the Stop Button
stopBtn = uicontrol('Parent', fig, 'Style', 'pushbutton', ...
    'String', 'Stop Simulation', ...
    'Units', 'normalized', ...
    'Position', [0.53, 0.02, 0.12, 0.05], ...  
    'FontSize', 12, 'FontWeight', 'bold', ...
    'BackgroundColor', [0.8 0.2 0.2], 'ForegroundColor', 'w', ...
    'Enable', 'off', ... 
    'Callback', @stopButtonCallback);

setappdata(fig, 'playBtn', playBtn);
setappdata(fig, 'stopBtn', stopBtn);

%% 2. Subplot Setup & Geometry
box_L = 0.45; box_W = 0.33; box_H = 0.25; 
ax_len = 0.4; % Length of the body-fixed axis lines
v_base = [ box_L/2,  box_W/2, -box_H/2; -box_L/2,  box_W/2, -box_H/2; -box_L/2, -box_W/2, -box_H/2; box_L/2, -box_W/2, -box_H/2;
           box_L/2,  box_W/2,  box_H/2; -box_L/2,  box_W/2,  box_H/2; -box_L/2, -box_W/2,  box_H/2; box_L/2, -box_W/2,  box_H/2];
faces = [1 2 3 4; 5 6 7 8; 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8];

% --- Subplot 1: 3D View (Top Left) ---
subplot(2, 2, 1);
hold on; grid on; view(3);
title('3D Trajectory');
xlabel('X (Forward) [m]'); ylabel('Y (Left) [m]'); zlabel('Z (Down) [m]');
set(gca, 'ZDir', 'reverse'); 
axis([xlims ylims zlims]);
daspect([1 1 1]); 

if is_static
    plot3([x(1), x_ref(1)], [y(1), y_ref(1)], [z(1), z_ref(1)], 'k--', 'LineWidth', 1.5);
    plot3(x_ref(1), y_ref(1), z_ref(1), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
else
    plot3(x_ref, y_ref, z_ref, 'k--', 'LineWidth', 1.5); % Background Reference Path
end

auv_patch = patch('Vertices', v_base, 'Faces', faces, 'FaceColor', [0.2 0.6 1], 'FaceAlpha', 0.8, 'EdgeColor', 'k');
path_line3d = plot3(nan, nan, nan, 'm-', 'LineWidth', 2.5);
vel_line3d = plot3(nan, nan, nan, 'm-', 'LineWidth', 2); 
bx_3d = plot3(nan, nan, nan, 'r-', 'LineWidth', 2); 
by_3d = plot3(nan, nan, nan, 'g-', 'LineWidth', 2); 
bz_3d = plot3(nan, nan, nan, 'b-', 'LineWidth', 2); 
plot3(x(1), y(1), z(1), 'go', 'MarkerFaceColor', 'g'); 
target_mark3d = plot3(nan, nan, nan, 'kx', 'MarkerSize', 10, 'LineWidth', 2); % Moving Target

% --- Subplot 2: XY Plane / Top View (Top Right) ---
subplot(2, 2, 2);
hold on; grid on; title('XY Plane (Top View)');
xlabel('X (Forward) [m]'); ylabel('Y (Left) [m]');
axis([xlims ylims]);
daspect([1 1 1]); 

if is_static
    plot([x(1), x_ref(1)], [y(1), y_ref(1)], 'k--', 'LineWidth', 1.5);
    plot(x_ref(1), y_ref(1), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
else
    plot(x_ref, y_ref, 'k--', 'LineWidth', 1.5); % Background Reference Path
end

auv_patch_xy = patch('Vertices', [v_base(:,1), v_base(:,2)], 'Faces', faces, 'FaceColor', [0.2 0.6 1], 'FaceAlpha', 0.4, 'EdgeColor', 'k');
path_xy = plot(nan, nan, 'm-', 'LineWidth', 2.5);
bx_xy = plot(nan, nan, 'r-', 'LineWidth', 2);
by_xy = plot(nan, nan, 'g-', 'LineWidth', 2);
bz_xy = plot(nan, nan, 'b-', 'LineWidth', 2);
pos_xy = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
plot(x(1), y(1), 'go', 'MarkerFaceColor', 'g');
target_mark_xy = plot(nan, nan, 'kx', 'MarkerSize', 10, 'LineWidth', 2); % Moving Target

% --- Subplot 3: XZ Plane / Side View (Bottom Left) ---
subplot(2, 2, 3);
hold on; grid on; title('XZ Plane (Side View)');
xlabel('X (Forward) [m]'); ylabel('Z (Down) [m]');
axis([xlims zlims]);
set(gca, 'YDir', 'reverse'); 
daspect([1 1 1]); 

if is_static
    plot([x(1), x_ref(1)], [z(1), z_ref(1)], 'k--', 'LineWidth', 1.5);
    plot(x_ref(1), z_ref(1), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
else
    plot(x_ref, z_ref, 'k--', 'LineWidth', 1.5); % Background Reference Path
end

auv_patch_xz = patch('Vertices', [v_base(:,1), v_base(:,3)], 'Faces', faces, 'FaceColor', [0.2 0.6 1], 'FaceAlpha', 0.4, 'EdgeColor', 'k');
path_xz = plot(nan, nan, 'm-', 'LineWidth', 2.5);
bx_xz = plot(nan, nan, 'r-', 'LineWidth', 2);
by_xz = plot(nan, nan, 'g-', 'LineWidth', 2);
bz_xz = plot(nan, nan, 'b-', 'LineWidth', 2);
pos_xz = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
plot(x(1), z(1), 'go', 'MarkerFaceColor', 'g');
target_mark_xz = plot(nan, nan, 'kx', 'MarkerSize', 10, 'LineWidth', 2); % Moving Target

% --- Subplot 4: YZ Plane / Front View (Bottom Right) ---
subplot(2, 2, 4);
hold on; grid on; title('YZ Plane (Front/Back View)');
xlabel('Y (Left) [m]'); ylabel('Z (Down) [m]');
axis([ylims zlims]);
set(gca, 'YDir', 'reverse'); 
daspect([1 1 1]); 

if is_static
    plot([y(1), y_ref(1)], [z(1), z_ref(1)], 'k--', 'LineWidth', 1.5);
    plot(y_ref(1), z_ref(1), 'kp', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'LineWidth', 1.5);
else
    plot(y_ref, z_ref, 'k--', 'LineWidth', 1.5); % Background Reference Path
end

auv_patch_yz = patch('Vertices', [v_base(:,2), v_base(:,3)], 'Faces', faces, 'FaceColor', [0.2 0.6 1], 'FaceAlpha', 0.4, 'EdgeColor', 'k');
path_yz = plot(nan, nan, 'm-', 'LineWidth', 2.5);
bx_yz = plot(nan, nan, 'r-', 'LineWidth', 2);
by_yz = plot(nan, nan, 'g-', 'LineWidth', 2);
bz_yz = plot(nan, nan, 'b-', 'LineWidth', 2);
pos_yz = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 4);
plot(y(1), z(1), 'go', 'MarkerFaceColor', 'g');
target_mark_yz = plot(nan, nan, 'kx', 'MarkerSize', 10, 'LineWidth', 2); % Moving Target

%% 3. Interactive Loop
sgtitle('Ready. Click Start Simulation to begin.', 'FontWeight', 'bold', 'FontSize', 14, 'Color', 'k');

while ishandle(fig)
    state = getappdata(fig, 'runState');
    
    if state == 0 || state == 2
        pause(0.1);
        
    elseif state == 1
        % Clear previous paths
        set(path_line3d, 'XData', nan, 'YData', nan, 'ZData', nan);
        set(path_xy, 'XData', nan, 'YData', nan);
        set(path_xz, 'XData', nan, 'YData', nan);
        set(path_yz, 'XData', nan, 'YData', nan);
        
        % Read requested playback speed
        speeds = getappdata(fig, 'speedFactors');
        menu = getappdata(fig, 'speedMenu');
        current_speed = speeds(menu.Value);
        
        t_start = tic; 
        t_elapsed = 0;
        i = 1;
        
        % Multiply real time by current_speed multiplier
        while t_elapsed < time(end) && ishandle(fig) && getappdata(fig, 'runState') == 1
            t_elapsed = toc(t_start) * current_speed;
            [~, i] = min(abs(time - t_elapsed));
            
            pos = [x(i); y(i); z(i)];
            r_phi = phi(i); r_theta = theta(i); r_psi = psi(i);
            V_body = [vel_u(i); vel_v(i); vel_w(i)];
            
            % Rotation Matrix
            R_rot = [cos(r_psi)*cos(r_theta), -sin(r_psi)*cos(r_phi)+cos(r_psi)*sin(r_theta)*sin(r_phi), sin(r_psi)*sin(r_phi)+cos(r_psi)*cos(r_phi)*sin(r_theta);
                 sin(r_psi)*cos(r_theta),  cos(r_psi)*cos(r_phi)+sin(r_phi)*sin(r_theta)*sin(r_psi), -cos(r_psi)*sin(r_phi)+sin(r_theta)*cos(r_phi)*sin(r_psi);
                 -sin(r_theta),            cos(r_theta)*sin(r_phi),                                  cos(r_theta)*cos(r_phi)];
            
            % 3D Translation & Rotation
            v_rot = (R_rot * v_base')'; 
            v_trans = v_rot + pos';
            
            % Body-fixed Axes Vectors
            Vx = pos + R_rot * [ax_len; 0; 0];
            Vy = pos + R_rot * [0; ax_len; 0];
            Vz = pos + R_rot * [0; 0; ax_len];
            
            % --- Update 3D Model & Axes ---
            set(auv_patch, 'Vertices', v_trans);
            set(path_line3d, 'XData', x(1:i), 'YData', y(1:i), 'ZData', z(1:i));
            
            set(bx_3d, 'XData', [pos(1), Vx(1)], 'YData', [pos(2), Vx(2)], 'ZData', [pos(3), Vx(3)]);
            set(by_3d, 'XData', [pos(1), Vy(1)], 'YData', [pos(2), Vy(2)], 'ZData', [pos(3), Vy(3)]);
            set(bz_3d, 'XData', [pos(1), Vz(1)], 'YData', [pos(2), Vz(2)], 'ZData', [pos(3), Vz(3)]);
            
            V_world = R_rot * V_body; 
            set(vel_line3d, 'XData', [pos(1), pos(1) + V_world(1)], 'YData', [pos(2), pos(2) + V_world(2)], 'ZData', [pos(3), pos(3) + V_world(3)]);
            
            % Update Moving Target Marker 3D
            set(target_mark3d, 'XData', x_ref(i), 'YData', y_ref(i), 'ZData', z_ref(i));
                            
            % --- Update 2D Projected Models & Axes ---
            % XY Plane
            set(auv_patch_xy, 'Vertices', [v_trans(:,1), v_trans(:,2)]);
            set(path_xy, 'XData', x(1:i), 'YData', y(1:i)); set(pos_xy, 'XData', pos(1), 'YData', pos(2));
            set(bx_xy, 'XData', [pos(1), Vx(1)], 'YData', [pos(2), Vx(2)]);
            set(by_xy, 'XData', [pos(1), Vy(1)], 'YData', [pos(2), Vy(2)]);
            set(bz_xy, 'XData', [pos(1), Vz(1)], 'YData', [pos(2), Vz(2)]);
            set(target_mark_xy, 'XData', x_ref(i), 'YData', y_ref(i)); % Target 
            
            % XZ Plane
            set(auv_patch_xz, 'Vertices', [v_trans(:,1), v_trans(:,3)]);
            set(path_xz, 'XData', x(1:i), 'YData', z(1:i)); set(pos_xz, 'XData', pos(1), 'YData', pos(3));
            set(bx_xz, 'XData', [pos(1), Vx(1)], 'YData', [pos(3), Vx(3)]);
            set(by_xz, 'XData', [pos(1), Vy(1)], 'YData', [pos(3), Vy(3)]);
            set(bz_xz, 'XData', [pos(1), Vz(1)], 'YData', [pos(3), Vz(3)]);
            set(target_mark_xz, 'XData', x_ref(i), 'YData', z_ref(i)); % Target 
            
            % YZ Plane
            set(auv_patch_yz, 'Vertices', [v_trans(:,2), v_trans(:,3)]);
            set(path_yz, 'XData', y(1:i), 'YData', z(1:i)); set(pos_yz, 'XData', pos(2), 'YData', pos(3));
            set(bx_yz, 'XData', [pos(2), Vx(2)], 'YData', [pos(3), Vx(3)]);
            set(by_yz, 'XData', [pos(2), Vy(2)], 'YData', [pos(3), Vy(3)]);
            set(bz_yz, 'XData', [pos(2), Vz(2)], 'YData', [pos(3), Vz(3)]);
            set(target_mark_yz, 'XData', y_ref(i), 'YData', z_ref(i)); % Target 
                          
            sgtitle(sprintf('Real-Time Playback: %.2f s | Position: (%.2f, %.2f, %.2f)', time(i), x(i), y(i), z(i)), 'FontWeight', 'bold', 'FontSize', 14, 'Color', 'k');
            drawnow; 
        end
        
        % --- Final Frame Lock ---
        if ishandle(fig)
            setappdata(fig, 'runState', 2); 
            playBtn.String = 'Restart Simulation';
            playBtn.BackgroundColor = [0.2 0.6 1]; 
            playBtn.Enable = 'on'; 
            stopBtn.Enable = 'off';
            menu.Enable = 'on'; % Re-enable speed selection
            
            sgtitle(sprintf('Manual Stop / Complete at %.2f s | Final Position: (%.2f, %.2f, %.2f)', time(i), x(i), y(i), z(i)), ...
                'FontWeight', 'bold', 'FontSize', 16, 'Color', [0 0.5 0]);
        end
    end
end

%% 4. Callback Functions
function playButtonCallback(src, ~)
    fig_obj = src.Parent;
    state = getappdata(fig_obj, 'runState');
    stopBtn = getappdata(fig_obj, 'stopBtn');
    speedMenu = getappdata(fig_obj, 'speedMenu');
    
    if state == 0 || state == 2
        setappdata(fig_obj, 'runState', 1); 
        src.String = 'Running...';
        src.BackgroundColor = [0.6 0.6 0.6]; 
        src.Enable = 'off'; 
        stopBtn.Enable = 'on'; 
        speedMenu.Enable = 'off'; % Lock speed selection during playback
    end
end

function stopButtonCallback(src, ~)
    fig_obj = src.Parent;
    setappdata(fig_obj, 'runState', 2); 
    src.Enable = 'off'; 
end
