clear all; clc; close all;

%% 轨迹生成（圆形下潜轨迹）
N = 20;         % 预测时域
T_sim = 30;     % 总仿真时间
dt = 0.001;       % 控制周期
num_steps = T_sim/dt;

% 参考轨迹参数
ref_traj = zeros(13, num_steps+N);
for k = 1:num_steps+N
    t = (k-1)*dt;
    % 位置（圆形+下潜）
    ref_traj(1:3,k) = [2*cos(0.5*t); 
                       2*sin(0.5*t); 
                       0.1*t];
    % 姿态（水平，四元数）
    ref_traj(4:7,k) = [1; 0; 0; 0];  % 实部在前
    % 速度（跟踪静止参考）
    ref_traj(8:13,k) = zeros(6,1);
end

%% acados OCP配置
model = BlueROV_heavy_model();  % 加载您的模型
ocp = AcadosOcp();

% 模型参数
ocp.model = model;

Q_pos = diag([1000, 1000, 1000]);       % 位置误差权重
Q_att = diag([60, 60, 60]);    % 姿态误差虚部权重
Q_vel = diag([10, 10, 10, 10, 10, 10]); % 速度误差权重
Q_fpos = 1000*Q_pos;       % 位置误差权重
Q_fatt = 10*Q_att;    % 姿态误差虚部权重
Q_fvel = 10*Q_vel; % 速度误差权重
R = diag(0.1*ones(8,1));          % 控制输入权重
cost_expr_ext_cost_custom_hess = blkdiag(R,Q_pos,0.1,Q_att,Q_vel);
cost_expr_ext_cost_custom_hess_e = blkdiag(Q_fpos,0.1,Q_fatt,Q_fvel);
ocp.model.cost_expr_ext_cost_custom_hess= cost_expr_ext_cost_custom_hess;
ocp.model.cost_expr_ext_cost_custom_hess_e= cost_expr_ext_cost_custom_hess_e;
% 代价函数参数
ocp.cost.cost_type = 'EXTERNAL';
ocp.cost.cost_type_e = 'EXTERNAL';

% 约束配置
x0 = [2.5;0.5;0;1;0;0;0;0;0;0;0;0;0];    % 初始状态
u_min = -40*ones(8,1);    % 推进器推力下限
u_max = 40*ones(8,1);     % 推进器推力上限
ocp.constraints.x0 = ref_traj(:,1);
ocp.constraints.lbu = u_min;
ocp.constraints.ubu = u_max;
ocp.constraints.idxbu = 0:7;

% 求解器选项
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
ocp.solver_options.hessian_approx = 'EXACT';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
ocp.solver_options.qp_solver_iter_max = 50;
ocp.solver_options.ext_fun_compile_flags = '-O3';
ocp.solver_options.nlp_solver_tol_stat = 1e-6;
ocp.solver_options.N_horizon =N;
ocp.solver_options.tf = N*0.05;

model_sim_method_num_stages = 4;
model_sim_method_num_steps = 1;

ocp.solver_options.sim_method_num_stages = model_sim_method_num_stages;
ocp.solver_options.sim_method_num_steps = model_sim_method_num_steps;


% 初始化求解器

ocp_solver = AcadosOcpSolver(ocp);


%% 仿真准备
X_sol = zeros(13, num_steps+1);
X_log = zeros(13, num_steps+1);  % 状态记录
U_log = zeros(8, num_steps);     % 控制输入记录
X_log(:,1) = x0;

cost_time = 0;

sim = AcadosSim();
sim.model = model;
sim.solver_options.Tsim = dt; % simulation time
sim.solver_options.integrator_type = 'ERK';        
plant_sim_method_num_stages = 4;
plant_sim_method_num_steps = 1;

sim.solver_options.num_stages = plant_sim_method_num_stages;
sim.solver_options.num_steps = plant_sim_method_num_steps;
% 构建模拟器
sim_solver = AcadosSimSolver(sim);
%% 修正后的主控制循环
for k = 1:num_steps
    % 滚动时域参数
    current_ref = ref_traj(:, k:min(k+50*N, end));
    
    % 逐节点设置参数（关键修正）
    for stage_idx = 0:N
        if 50*stage_idx+1 <= size(current_ref,2)
            ocp_solver.set( 'p', current_ref(:,50*stage_idx+1),stage_idx);
        else
            % 超出部分使用最后已知参考值
            ocp_solver.set( 'p', current_ref(:,end),stage_idx);
        end
    end

    % 设置初始状态,
    ocp_solver.set('x', X_log(:,k),0);
    ocp_solver.set('constr_x0',  X_log(:,k));
    tic
    ocp_solver.solve();  % 直接调用，无输出参数
    toc
     % disp(['运行时间: ',num2str(toc)]);
        cost_time = [cost_time;toc];
    status = ocp_solver.get('status'); % 0 - success

    % 获取控制量
    
    u_opt = ocp_solver.get( 'u',0);
    X_sol(:,k) = ocp_solver.get( 'x',0);
    U_log(:,k) = u_opt;
    
    % 使用 acados 模拟器
    sim_solver.set('x', X_log(:,k));
    sim_solver.set('u', U_log(:,k));

    
    % 执行模拟
    sim_status = sim_solver.solve();
    
    % 获取下一个状态
    X_log(:,k+1) = sim_solver.get('xn');
end
     disp(['平均计算时间: ',num2str(norm(cost_time,1)/num_steps)]);
     disp(['最大计算时间: ',num2str(norm(cost_time,Inf))]);
%% 可视化结果
figure;
subplot(3,1,1);
plot3(X_log(1,:), X_log(2,:), X_log(3,:), 'b'); hold on;
plot3(ref_traj(1,1:num_steps), ref_traj(2,1:num_steps), ref_traj(3,1:num_steps), 'r--');
title('3D轨迹跟踪');
legend('实际轨迹','参考轨迹');

subplot(3,1,2);
plot(0:dt:T_sim, X_log(3,:)); 
title('深度跟踪');
xlabel('时间(s)'); ylabel('Z位置(m)');

subplot(3,1,3);
stairs(0:dt:T_sim-dt, U_log');
title('推进器控制输入');
xlabel('时间(s)'); ylabel('推力(N)');
