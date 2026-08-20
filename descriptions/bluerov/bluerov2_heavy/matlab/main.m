clear all; clc; close all;
ocp = AcadosOcp();
%% Trajectory generation: circular path at fixed depth
N = 20;         % Prediction horizon
T_sim = 20;     % Total simulation time
dt = 0.001;     % Control period
num_steps = T_sim/dt;
kt = 25;
num_steps = num_steps+N*kt;
shooting_nodes = [0.0,dt*kt*(1:N)];
T = shooting_nodes(end);
radius = 5;
% Reference trajectory parameters.
ref_traj = zeros(13, num_steps);
yaw_log=[];
for k = 1:num_steps
    t = (k-1)*dt;
    if k>num_steps-N*kt
        t = (num_steps-1)*dt;
        ref_traj(8:13,k) = [0;
            0;
            0;0;0;0];
    end
    % Circular position at fixed depth.
    omega = 0.15;
    ref_traj(1:3,k) = [
        2.0*sin(omega*t);
        -2.0*cos(omega*t);
        0.5];
    vx = omega*2.0*cos(omega*t);
    vy = omega*2.0*sin(omega*t);

    % Desired yaw follows the path tangent.
    yaw = atan2(vy, vx);
    % NED quaternion from ZYX Euler angles: roll=0, pitch=0, yaw=yaw.
    % q = [cos(psi/2); 0; 0; sin(psi/2)]
    ref_traj(4:7,k) = [cos(yaw/2); 0; 0; sin(yaw/2)];

    % Body-frame reference velocity: forward speed only.
    ref_traj(8:13,k) = [sqrt(vx*vx+vy*vy);0;0;0;0;0];

end

%% acados OCP configuration
model = BlueROV_heavy_model();      % Load OCP model
sim_model = BlueROV_heavy_model_sim();  % Simulation model with thrust health in p


ocp.model = model;

% Cost configuration.
ocp.cost.cost_type = 'EXTERNAL';
ocp.cost.cost_type_e = 'EXTERNAL';

% Constraint configuration.
x0 = [0;-2;0.5;1;0;0;0;0;0;0;0;0;0];    % Initial state
u_barr = [40;40;40;40;40;40;40;40;];
u_min = -u_barr;    % Thruster lower bound
u_max = u_barr;     % Thruster upper bound
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
X_sol = zeros(13, num_steps+1);
X_log = zeros(13, num_steps+1);  % State log
U_log = zeros(8, num_steps);     % Control input log
X_log(:,1) = x0;

cost_time = 0;

sim = AcadosSim();
sim.model = sim_model;
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
            ocp_solver.set('p', current_ref(:,stage_idx*kt+1), stage_idx);
        else
            % Use the last known reference when the horizon extends past the end.
            ocp_solver.set('p', current_ref(:,end), stage_idx);
        end

    end

    ocp_solver.set('constr_x0', X_log(:,k));
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
plot3(X_log(1,:), X_log(2,:), X_log(3,:), 'b', 'LineWidth', 2); hold on;
plot3(ref_traj(1,1:num_steps), ref_traj(2,1:num_steps), ref_traj(3,1:num_steps), 'r--', 'LineWidth', 1.5);

grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D Trajectory Tracking');
legend('Actual trajectory', 'Reference trajectory', 'Location', 'best');
