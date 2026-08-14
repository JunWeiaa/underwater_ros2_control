clear all; clc; close all;
ocp = AcadosOcp();
%% 轨迹生成（圆形下潜轨迹）
N = 20;         % 预测时域
T_sim = 20;     % 总仿真时间
dt = 0.001;     % 控制周期
num_steps = T_sim/dt;
kt = 25;
num_steps = num_steps+N*kt;
shooting_nodes = [0.0,dt*kt*(1:N)];
T = shooting_nodes(end);
radius = 5;
% 参考轨迹参数
ref_traj = zeros(13, num_steps);
yaw_log=[];
for k = 1:num_steps
    %  螺旋
    t = (k-1)*dt;
    % ref_traj(8:13,k) = [-sin(0.5*t);
    %                cos(0.5*t);
    %                0.1;0;0;0];
    if k>num_steps-N*kt
        t = (num_steps-1)*dt;
        ref_traj(8:13,k) = [0;
            0;
            0;0;0;0];
    end
    % 位置（圆形+下潜）
    omega = 0.15;
    ref_traj(1:3,k) = [
        2.0*sin(omega*t);
        -2.0*cos(omega*t);
        0.5];
    vx = omega*2.0*cos(omega*t);
    vy = omega*2.0*sin(omega*t);

    % 计算期望偏航角 (沿着切线方向)
    yaw = atan2(vy, vx);
    % NED坐标系四元数转换 (ZYX Euler: Roll=0, Pitch=0, Yaw=yaw)
    % q = [cos(psi/2); 0; 0; sin(psi/2)]
    ref_traj(4:7,k) = [cos(yaw/2); 0; 0; sin(yaw/2)];

    % 参考速度 (Body Frame): 只有前向速度 u, v=w=0
    ref_traj(8:13,k) = [sqrt(vx*vx+vy*vy);0;0;0;0;0];
    % 单点
    % ref_traj(8:13,k) = [0;
    %     0;
    %     0;0;0;0];
    % if k>num_steps-N*kt
    %     t = (num_steps-1)*dt;
    %     ref_traj(8:13,k) = [0;
    %         0;
    %         0;0;0;0];
    % end
    % % 位置（圆形+下潜）
    % ref_traj(1:3,k) = [0;
    %     0;
    %     0.5];
    % 姿态（水平，四元数）
    % ref_traj(4:7,k) = [1; 0; 0; 0];  % 实部在前
    % 8字
    % % dx = 0.1*radius * cos(0.1*t);
    % % dy = 0.2*radius * cos(0.2*t);
    % % if dx>0.1
    % %     dx = 0.1;
    % % elseif dx<-0.1
    % %     dx = -0.1;
    % % else
    % %     dx = 0;
    % % end
    % % if dy>0.1
    % %     dy = 0.1;
    % % elseif dy<-0.1
    % %     dy = -0.1;
    % % else
    % %     dy = 0;
    % % end
    %
    % ref_traj(8:13,k) = [-30;
    %     0;
    %     0;0;0;0];
    % if k>num_steps-N*kt
    %     t = (num_steps-1)*dt;
    %     ref_traj(8:13,k) = [0;
    %         0;
    %         0;0;0;0];
    % end
    % ref_traj(1:3,k) = [radius * sin(0.1*t);
    %     radius * sin(0.2*t); % 双倍频率形成8字
    %     0.5];
    % %     yaw = atan2(dy, dx);
    % % yaw_log = [yaw_log;yaw];
    % % ref_traj(4:7,k) = eul2quat([yaw,0,0],'ZYX')';
    % ref_traj(4:7,k) = [1; 0; 0; 0];  % 实部在前
    % 速度（跟踪静止参考）

end

%% acados OCP配置
model = BlueROV_heavy_model();  % 加载OCP模型
sim_model = BlueROV_heavy_model_sim();  % 仿真模型（p含thrust_health）


% 模型参数
ocp.model = model;

% Q_pos = diag([80, 80, 80]);       % 位置误差权重
% Q_att = diag([800, 800, 800]);    % 姿态误差虚部权重
% Q_vel = diag([1, 1, 1, 0.5, 0.5, 0.1]); % 速度误差权重
% Q_fpos = 10*Q_pos;       % 位置误差权重
% Q_fatt = 10*Q_att;    % 姿态误差虚部权重
% Q_fvel = 10*Q_vel; % 速度误差权重
% R = diag(0.1*ones(8,1));          % 控制输入权重
% cost_expr_ext_cost_custom_hess = blkdiag(R,Q_pos,0,Q_att,Q_vel);
% cost_expr_ext_cost_custom_hess_e = blkdiag(Q_fpos,0,Q_fatt,Q_fvel);
% ocp.model.cost_expr_ext_cost_custom_hess= cost_expr_ext_cost_custom_hess;
% ocp.model.cost_expr_ext_cost_custom_hess_e= cost_expr_ext_cost_custom_hess_e;
% 代价函数参数
ocp.cost.cost_type = 'EXTERNAL';
ocp.cost.cost_type_e = 'EXTERNAL';

% 约束配置
x0 = [0;-2;0.5;1;0;0;0;0;0;0;0;0;0];    % 初始状态
% thrust_health = [0.5, 0.5, 0.5, 0.5, 1, 1, 0, 1]; % 推进器健康状态
u_barr = [40;40;40;40;40;40;40;40;];
u_min = -u_barr;    % 推进器推力下限
u_max = u_barr;     % 推进器推力上限
ocp.constraints.x0 = x0;
ocp.constraints.lbu = u_min;
ocp.constraints.ubu = u_max;
ocp.constraints.idxbu = 0:7;
% ocp.constraints.lh_e = [0; 0];
% ocp.constraints.uh_e = [100;1];
% 重新启用约束，但使用非常宽松的边界进行测试
% 测试4个简单的线性位置约束
% ocp.constraints.lh = [0; 0; 0;0;0;0];  % 下界为0（约束应该>=0）
% ocp.constraints.uh = [10000000; 10000000; 10000000;10000000; 10000000;10000000];    % 上界设为无穷大


% 求解器选项
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM';
% ocp.solver_options.hessian_approx = 'EXACT';
ocp.solver_options.hessian_approx = 'GAUSS_NEWTON';
ocp.solver_options.integrator_type = 'ERK';
ocp.solver_options.shooting_nodes = shooting_nodes;
ocp.solver_options.nlp_solver_type = 'SQP_RTI';
% ocp.solver_options.nlp_solver_type = 'SQP_WITH_FEASIBLE_QP';
%
% ocp.solver_options.search_direction_mode = 'BYRD_OMOJOKUN';
% ocp.solver_options.allow_direction_mode_switch_to_nominal = false;
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


% 初始化求解器

ocp_solver = AcadosOcpSolver(ocp);


%% 仿真准备
X_sol = zeros(13, num_steps+1);
X_log = zeros(13, num_steps+1);  % 状态记录
U_log = zeros(8, num_steps);     % 控制输入记录
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
% 构建模拟器
sim_solver = AcadosSimSolver(sim);
%% 修正后的主控制循环

for k = 1:num_steps
    % 滚动时域参数
    current_ref = ref_traj(:, k:min(k*kt, end));

    % 逐节点设置参数（关键修正）
    for stage_idx = 0:N
        if stage_idx*kt+1 <= size(current_ref,2)
            ocp_solver.set('p', current_ref(:,stage_idx*kt+1), stage_idx);
        else
            % 超出部分使用最后已知参考值
            ocp_solver.set('p', current_ref(:,end), stage_idx);
        end

    end
    % % ocp_solver.set('p_global', thrust_health'); % 确保传入列向量
    % 设置初始状态,

    ocp_solver.set('constr_x0', X_log(:,k));
    tic
    ocp_solver.solve();  % 直接调用，无输出参数
    toc
    % disp(['运行时间: ',num2str(toc)]);
    cost_time = [cost_time;toc];
    status = ocp_solver.get('status'); % 0 - success

    % 获取控制量
    if  status==0
        u_opt = ocp_solver.get('u',0);
    else
        disp('计算出错');
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

    % 使用 acados 模拟器
    sim_solver.set('x', X_log(:,k));
    sim_solver.set('u', U_log(:,k));
    % 仿真模型使用 p = [ref_path; thrust_health]


    % 执行模拟
    sim_status = sim_solver.solve();

    % 获取下一个状态
    X_log(:,k+1) = sim_solver.get('xn');

end
disp(['平均计算时间: ',num2str(norm(cost_time,1)/num_steps)]);
disp(['最大计算时间: ',num2str(norm(cost_time,Inf))]);
%% 可视化结果
figure;
plot3(X_log(1,:), X_log(2,:), X_log(3,:), 'b', 'LineWidth', 2); hold on;
plot3(ref_traj(1,1:num_steps), ref_traj(2,1:num_steps), ref_traj(3,1:num_steps), 'r--', 'LineWidth', 1.5);

grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D轨迹跟踪');
legend('实际轨迹', '参考轨迹', 'Location', 'best');

% subplot(3,1,2);
% plot(0:dt:T_sim, X_log(3,:));
% title('深度跟踪');
% xlabel('时间(s)'); ylabel('Z位置(m)');
%
% subplot(3,1,3);
% stairs(0:dt:T_sim-dt, U_log');
% title('推进器控制输入');
% xlabel('时间(s)'); ylabel('推力(N)');
