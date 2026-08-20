clear all; clc; close all;
ocp = AcadosOcp();
%% Trajectory generation: circular path at fixed depth
N = 20;         % Prediction horizon
T_sim = 30;     % Total simulation time
dt = 0.005;     % Control period
num_steps = T_sim/dt;
kt = 10;
num_steps = num_steps+N*kt;
shooting_nodes = [0.0,dt*kt*(1:N)];
T = shooting_nodes(end);
% Reference trajectory parameters.
ref_traj = zeros(13, num_steps);
for k = 1:num_steps
    t = (k-1)*dt;
    ref_traj(8:13,k) = [0;
        0;
        0;0;0;0];
    if k>num_steps-N*kt
        t = (num_steps-1)*dt;
        ref_traj(8:13,k) = [0;
            0;
            0;0;0;0];
    end
    % Circular position at fixed depth.
    ref_traj(1:3,k) = [2*cos(0.2*t)-2;
        2*sin(0.2*t);
        0.5];

    % Motion direction from planar velocity.
    dx = -0.4*sin(0.2*t);  % x velocity
    dy = 0.4*cos(0.2*t);   % y velocity

    % Yaw points the vehicle nose along the motion direction.
    yaw_angle = atan2(dy, dx);

    % Convert yaw to quaternion; only qw and qz are nonzero.
    ref_traj(4:7,k) = [cos(yaw_angle/2); 0; 0; sin(yaw_angle/2)];
end

%% acados OCP configuration
model = subcat_model();


ocp.model = model;

% Cost configuration.
ocp.cost.cost_type = 'EXTERNAL';
ocp.cost.cost_type_e = 'EXTERNAL';

% Constraint configuration.
x0 = [0;0;0.5;0.707;0;0;0.707;0;0;0;0;0;0;0;0;0;0];    % Initial state
thrust_health = [1, 1, 1, 1, 1, 1, 1, 1]; % Thruster health state
u_barr = [20;20;20;20;0.175;0.175;0.175;0.175;];
u_min = -diag(thrust_health)*u_barr;    % Lower control bounds
u_max = diag(thrust_health)*u_barr;     % Upper control bounds
ocp.constraints.x0 = x0;
ocp.constraints.lbu = u_min;
ocp.constraints.ubu = u_max;
ocp.constraints.idxbu = 0:7;


% Solver options.
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.shooting_nodes = shooting_nodes;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.qp_solver_iter_max = 50;
ocp.solver_options.ext_fun_compile_flags = '-O3';
ocp.solver_options.nlp_solver_tol_stat = 1e-6;
ocp.solver_options.N_horizon =N;
ocp.solver_options.tf = T;
ocp.solver_options.regularize_method = 'PROJECT';
model_sim_method_num_stages = 2;
model_sim_method_num_steps = 1;

ocp.solver_options.sim_method_num_stages = model_sim_method_num_stages;
ocp.solver_options.sim_method_num_steps = model_sim_method_num_steps;


% Initialize solver.

ocp_solver = AcadosOcpSolver(ocp);


%% Simulation setup
X_sol = zeros(17, num_steps+1);
X_log = zeros(17, num_steps+1);  % State log
U_log = zeros(8, num_steps);     % Control input log
X_log(:,1) = x0;

cost_time = 0;

sim = AcadosSim();
sim.model = model;
% sim.model.p_global=[];
sim.solver_options.Tsim = dt; % simulation time
sim.solver_options.integrator_type = 'ERK';
plant_sim_method_num_stages = 4;
plant_sim_method_num_steps = 1;

sim.solver_options.num_stages = plant_sim_method_num_stages;
sim.solver_options.num_steps = plant_sim_method_num_steps;
sim_solver = AcadosSimSolver(sim);
%% Main receding-horizon loop

for k = 1:num_steps
    % Rolling-horizon parameters.
    current_ref = ref_traj(:, k:min(k*kt, end));

    % Set parameters at each shooting node.
    for stage_idx = 0:N
        if stage_idx*kt+1 <= size(current_ref,2)
            ocp_solver.set( 'p', current_ref(:,stage_idx*kt+1),stage_idx);
        else
            % Use the last known reference when the horizon extends past the end.
            ocp_solver.set( 'p', current_ref(:,end),stage_idx);
        end

    end
    ocp_solver.set('constr_x0',  X_log(:,k));
    tic
    ocp_solver.solve();
    toc
    cost_time = [cost_time;toc];
    status = ocp_solver.get('status'); % 0 - success

    if  status==0
        u_opt = ocp_solver.get('u',0);
    else
        disp('Solver failed');
        % ocp_solver.get( 'h',0)
        if k~=1
            u_opt = U_log(:,k-1);
        else
            break;
        end
        % u_opt = 0;

        break;
    end

    % X_sol(:,k) = ocp_solver.get( 'x',0);
    U_log(:,k) = u_opt;

    sim_solver.set('x', X_log(:,k));
    sim_solver.set('u', U_log(:,k));


    sim_status = sim_solver.solve();

    X_log(:,k+1) = sim_solver.get('xn');

end
disp(['Average compute time: ',num2str(norm(cost_time,1)/num_steps)]);
disp(['Max compute time: ',num2str(norm(cost_time,Inf))]);
%% Visualization
figure;
subplot(3,1,1);
plot3(X_log(1,:), X_log(2,:), X_log(3,:), 'b', 'LineWidth', 2); hold on;
plot3(ref_traj(1,1:num_steps), ref_traj(2,1:num_steps), ref_traj(3,1:num_steps), 'r--', 'LineWidth', 1.5);

% Safety boundary display.
R_cylinder = 4.0; % Cylinder radius
H_cylinder = 1.6; % Cylinder height
d_safe = 0.1;     % Safety distance

% Cylindrical safety boundary.
theta = linspace(0, 2*pi, 100);
r_safe = R_cylinder - d_safe;  % Safe radius

% Bottom circle, z = d_safe.
x_circle_bottom = r_safe * cos(theta);
y_circle_bottom = r_safe * sin(theta);
z_circle_bottom = d_safe * ones(size(theta));
plot3(x_circle_bottom, y_circle_bottom, z_circle_bottom, 'g--', 'LineWidth', 2);

% Top circle, z = H_cylinder - d_safe.
z_top = H_cylinder - d_safe;
z_circle_top = z_top * ones(size(theta));
plot3(x_circle_bottom, y_circle_bottom, z_circle_top, 'g--', 'LineWidth', 2);

% Vertical side lines.
num_lines = 8;
theta_lines = linspace(0, 2*pi, num_lines+1);
theta_lines = theta_lines(1:end-1);
for i = 1:num_lines
    x_line = r_safe * cos(theta_lines(i)) * [1; 1];
    y_line = r_safe * sin(theta_lines(i)) * [1; 1];
    z_line = [d_safe; z_top];
    plot3(x_line, y_line, z_line, 'g--', 'LineWidth', 1);
end

grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Trajectory Tracking With Safety Boundary');
legend('Actual trajectory', 'Reference trajectory', 'Safety boundary', 'Location', 'best');

axis equal;
xlim([-5, 5]);
ylim([-5, 5]);
zlim([0, 2]);
view(45, 30);

subplot(3,2,3);
plot(X_log(1,:), X_log(2,:), 'b', 'LineWidth', 2); hold on;
plot(ref_traj(1,1:num_steps), ref_traj(2,1:num_steps), 'r--', 'LineWidth', 1.5);
plot(x_circle_bottom, y_circle_bottom, 'g--', 'LineWidth', 2);
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('XY Trajectory Projection');
legend('Actual trajectory', 'Reference trajectory', 'Safety boundary', 'Location', 'best');
axis equal;

subplot(3,2,4);
time_vec = (0:length(X_log(3,:))-1) * dt;
plot(time_vec, X_log(3,:), 'b', 'LineWidth', 2); hold on;
plot((0:num_steps-1)*dt, ref_traj(3,1:num_steps), 'r--', 'LineWidth', 1.5);
plot([0, time_vec(end)], [d_safe, d_safe], 'g--', 'LineWidth', 2);
plot([0, time_vec(end)], [z_top, z_top], 'g--', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Depth Z (m)');
title('Depth Profile');
legend('Actual depth', 'Reference depth', 'Safety boundary', 'Location', 'best');

subplot(3,2,5);
r_actual = sqrt(X_log(1,:).^2 + X_log(2,:).^2);
radial_margin = r_safe - r_actual;
plot(time_vec, radial_margin, 'b', 'LineWidth', 2); hold on;
plot([0, time_vec(end)], [0, 0], 'r--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Radial safety margin (m)');
title('Radial Safety Margin');
legend('Safety margin', 'Boundary', 'Location', 'best');

subplot(3,2,6);
depth_margin_lower = X_log(3,:) - d_safe;
depth_margin_upper = z_top - X_log(3,:);
plot(time_vec, depth_margin_lower, 'b', 'LineWidth', 2); hold on;
plot(time_vec, depth_margin_upper, 'g', 'LineWidth', 2);
plot([0, time_vec(end)], [0, 0], 'r--', 'LineWidth', 1);
grid on;
xlabel('Time (s)');
ylabel('Depth safety margin (m)');
title('Depth Safety Margin');
legend('Lower margin', 'Upper margin', 'Boundary', 'Location', 'best');
