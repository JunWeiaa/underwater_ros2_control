%% 清空环境
clear all; clc; close all;



import casadi.*

% options needed for the Simulink example
if ~exist('simulink_opts','var')
    % disp('using acados simulink default options')
    % simulink_opts = get_acados_simulink_opts;
    disp('using empty simulink_opts to generate solver without simulink block')
    simulink_opts = [];
end

%
check_acados_requirements()

%% solver settings
N = 60; % number of discretization steps
T = 3; % [s] prediction horizon length
x0 = [0;0;0;1;0;0;0;0;0;0; 0; 0;0]; % initial state


A = 0.2;        % 基础幅度（保证最大跨度为1m）
dt = 0.05;      % 时间间隔
t = 0:dt:T;     % 时间序列


%% 轨迹生成（XY平面8字）
x1 = A*sin(2*pi*t);          % X轴轨迹
y1 = A*sin(4*pi*t);          % Y轴轨迹
z1 = ones(1,N+1);             % Z轴固定

%% 速度计算
u1 = A*pi*cos(2*pi*t);%pi*cos(2*pi*t);         % X方向速度
v1 = 2*A*pi*cos(4*pi*t);%2*pi*cos(4*pi*t);       % Y方向速度
w1 = zeros(1,N+1);             % Z方向速度

%% 姿态计算（零滚转和俯仰）
theta1 = unwrap(atan2(v1, u1));% 解包连续偏航角
q01 =cos(theta1/2);%cos(theta1/2);          % 四元数分量
q31 = sin(theta1/2);%sin(theta1/2);
q11 = zeros(1,N+1);
q21 = zeros(1,N+1);
q_norm = sqrt(q01.^2 + q11.^2 + q21.^2 + q31.^2);
q01 = q01 ./ q_norm;
q11 = q11 ./ q_norm;
q21 = q21 ./ q_norm;
q31 = q31 ./ q_norm;
%% 角速度计算（中心差分法）
r1 = zeros(1,N+1);             % 偏航角速度
% r1(1) = (theta1(2)-theta1(1))/dt;
% r1(end) = (theta1(end)-theta1(end-1))/dt;
for i = 2:N
    r1(i) = 0;%(theta1(i+1)-theta1(i-1))/(2*dt);
end
p1 = zeros(1,N+1);             % 滚转角速度
q1 = zeros(1,N+1);             % 俯仰角速度

%% 结果整合
x_ref = [x1; y1; z1; q01; q11; q21; q31; u1; v1; w1; p1; q1; r1];
x_ref = [x_ref,x_ref(:,end)];
% for i = 1:N+2
%     x_ref(:,i) = [3;-2;1;1;0;0;0;1;1;0; 0; 0;0];
% end


%% model dynamics
model = BlueROV_heavy_model();
nx = length(model.x); % state size
nu = length(model.u); % input size

%% OCP formulation objects
ocp = AcadosOcp();
ocp.model = BlueROV_heavy_model();
x = ocp.model.x;
u = ocp.model.u;
%% cost in nonlinear least squares form

ocp.cost.cost_type = 'EXTERNAL';
% final cost term
ocp.cost.cost_type_e = 'EXTERNAL';
%% 代价函数参数
Q_pos = diag([80, 80, 80]);       % 位置误差权重
Q_att = diag([60, 60, 80]);    % 姿态误差虚部权重
Q_vel = diag([1, 1, 1, 0.5, 0.5, 1]); % 速度误差权重
Q_fpos = 1*Q_pos;       % 位置误差权重
Q_fatt = 1*Q_att;    % 姿态误差虚部权重
Q_fvel = 1*Q_vel; % 速度误差权重
R = diag(0.1*ones(8,1));          % 控制输入权重
% cost_expr_ext_cost_custom_hess = blkdiag(R,Q_pos,0,Q_att,Q_vel);
% cost_expr_ext_cost_custom_hess_e = blkdiag(Q_fpos,0,Q_fatt,Q_fvel);
% ocp.model.cost_expr_ext_cost_custom_hess= cost_expr_ext_cost_custom_hess;
% ocp.model.cost_expr_ext_cost_custom_hess_e= cost_expr_ext_cost_custom_hess_e;
%% define constraints
% only bound on u on initial stage and path
ocp.model.con_h_expr =[x;norm(x(4:7))-1; u];
ocp.model.con_h_expr_0 = [x;norm(x(4:7))-1;u];

U_max = 40*ones(nu,1);
X_max = [10;10;10;1;1;1;1;10;10;10;10;10;10];
ocp.constraints.lh = -[X_max;0;U_max];
ocp.constraints.lh_0 = -[X_max;0;U_max];
ocp.constraints.uh = [X_max;0;U_max];
ocp.constraints.uh_0 = [X_max;0;U_max];
ocp.constraints.x0 = x_ref(:,1);

% define solver options
ocp.solver_options.N_horizon = N;
ocp.solver_options.tf = T;
ocp.solver_options.nlp_solver_type = 'SQP';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.qp_solver_mu0 = 1e3;
ocp.solver_options.qp_solver_cond_N = 50;
ocp.solver_options.hessian_approx = 'EXACT';
ocp.solver_options.ext_fun_compile_flags = '-O3';
ocp.solver_options.globalization = 'MERIT_BACKTRACKING';
% ocp.solver_options.qp_solver_iter_max = 100
ocp.simulink_opts = simulink_opts;
%% 增强数值稳定性
ocp.solver_options.nlp_solver_max_iter = 300;
ocp.solver_options.qp_solver_iter_max = 300;

ocp.solver_options.nlp_solver_tol_stat = 1e-4;
% create solver


ocp_solver = AcadosOcpSolver(ocp);

% for  i = 0:N
%     ocp_solver.set('p',x_ref(:,i+1),i);
% end
% 
% for i=0:N
%     p_i = ocp_solver.get('p',i);
%     fprintf('Step %2d 参考位置: [%.2f, %.2f, %.2f]\n',...
%             i, p_i(1), p_i(2), p_i(3));
% end
% solver initial guess
x_traj_init = x_ref(:,1);
u_traj_init = zeros(nu, N);
for i = 1:N+1
    x_traj_init(:,i) =  x_ref(:,1);
end

% call ocp solver
% update initial state
% ocp_solver.set('constr_x0',  x_ref(:,1));
% 
% ocp_solver.set('constr_lh', ocp.constraints.lh )
% ocp_solver.set('constr_uh', ocp.constraints.uh )
% % % set trajectory initialization
ocp_solver.set('init_x', x_traj_init); % states
ocp_solver.set('init_u', u_traj_init); % inputs
ocp_solver.set('init_pi', zeros(nx, N)); % multipliers for dynamics equality constraints
% wwww = ocp_solver.get('stat',4);
for  i = 0:N
    ocp_solver.set( 'p', x_ref(:,i+1),i);
end
for i=0:N
    p_i = ocp_solver.get('p',i);
    fprintf('Step %2d 参考位置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n',...
            i, p_i(1), p_i(2), p_i(3), p_i(4), p_i(5), p_i(6));
end


% change values for specific shooting node using:
%   ocp_solver.set('field', value, optional: stage_index)
tic
% solve
ocp_solver.solve();
toc
disp(['运行时间: ',num2str(toc)]);
% get solution
utraj = ocp_solver.get('u');
xtraj = ocp_solver.get('x');

status = ocp_solver.get('status'); % 0 - success
ocp_solver.print('stat')

%% plots
ts = linspace(0, T, N+1);
figure; hold on;
states = {'x', 'y', 'z', 'q1','q2', 'q3', 'q4', 'vx','vy', 'vz', 'wx', 'wy','wz'};
for i=1:13
    subplot(length(states), 1, i);
    plot(ts, xtraj(i,:)); grid on;
    ylabel(states{i});
    xlabel('t [s]')
end

figure
stairs(ts, [utraj(1,:), utraj(1,end)])
ylabel('F [N]')
xlabel('t [s]')
grid on


figure('Name','三维轨迹跟踪');
plot3(x_ref(1,:), x_ref(2,:), x_ref(3,:), 'r--', 'LineWidth',2); hold on;
plot3(xtraj(1,:), xtraj(2,:), xtraj(3,:), 'b-', 'LineWidth',1.5);
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('参考轨迹', '预测轨迹');
grid on; axis equal;