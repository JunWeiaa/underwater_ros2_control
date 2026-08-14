clear all; clc; close all;
ocp = AcadosOcp();
%% 轨迹生成（圆形下潜轨迹）
N = 20;         % 预测时域
T_sim = 45;     % 总仿真时间
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
thrust_health = [0.5, 0.5, 0.5, 0.5, 1, 1, 0, 1]; % 推进器健康状态
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
ocp.constraints.lh = [0; 0; 0;0;0;0];  % 下界为0（约束应该>=0）
ocp.constraints.uh = [10000000; 10000000; 10000000;10000000; 10000000;10000000];    % 上界设为无穷大


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
    ocp_solver.set('p_global', thrust_health'); % 确保传入列向量
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
    sim_solver.set('p', [current_ref(:,1); thrust_health(:)]);

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
plot3(X_log(1,:), X_log(2,:), X_log(3,:), 'b', 'LineWidth', 2); hold on;
plot3(ref_traj(1,1:num_steps), ref_traj(2,1:num_steps), ref_traj(3,1:num_steps), 'r--', 'LineWidth', 1.5);

% 添加安全边界显示
R_cylinder = 4.0; % 物理障碍物半径 (Wall)
H_cylinder = 1.6; % 物理障碍物高度
d_safe = 0.7;     % 安全缓冲距离 (Buffer) - 需与模型一致

% 绘制圆柱形安全边界 (使用曲面 surf 更显眼)
[X_cyl, Y_cyl, Z_cyl] = cylinder(R_cylinder, 100);
Z_cyl = Z_cyl * H_cylinder; % 高度拉伸

% 1. 绘制物理墙 (红色半透明墙体)
surf(X_cyl, Y_cyl, Z_cyl, 'FaceColor', 'r', 'FaceAlpha', 0.1, 'EdgeColor', 'r', 'EdgeAlpha', 0.3);
hold on;

% 2. 绘制安全缓冲界 (绿色线框)
[X_safe_cyl, Y_safe_cyl, Z_safe_cyl] = cylinder(R_cylinder - d_safe, 100);
Z_safe_cyl = Z_safe_cyl * H_cylinder;
surf(X_safe_cyl, Y_safe_cyl, Z_safe_cyl, 'FaceColor', 'g', 'FaceAlpha', 0.05, 'EdgeColor', 'g', 'EdgeAlpha', 0.2, 'LineStyle', '--');

% --- 新增可视化：障碍物 (h4) ---
% 定义障碍物参数（需与 Model 中 consistent）
p_obs_list = [sqrt(3)/2*2.1, -1.05, 0.5;
    0, 2.1, 0.5;
    -sqrt(3)/2*2.1, -1.05, 0.5];
r_safe_obs = 0.6; % 障碍物安全半径

% 绘制障碍物圆柱
[X_base, Y_base, Z_base] = cylinder(r_safe_obs, 50);
Z_base = Z_base * 1.5; % 让它足够高

for i = 1:size(p_obs_list, 1)
    p_curr = p_obs_list(i, :);
    X_obs = X_base + p_curr(1);
    Y_obs = Y_base + p_curr(2);
    % 用醒目的洋红色(Magenta)填充
    surf(X_obs, Y_obs, Z_base, 'FaceColor', 'm', 'FaceAlpha', 0.6, 'EdgeColor', 'k', 'EdgeAlpha', 0.2);
    text(p_curr(1), p_curr(2), 2.0, ['OBS', num2str(i)], 'Color', 'm', 'FontSize', 12, 'FontWeight', 'bold');
end
% --- 结束新增 ---

% 重新定义部分变量以修复后续绘图报错
r_safe = R_cylinder - d_safe;
theta = linspace(0, 2*pi, 100);
x_wall = R_cylinder * cos(theta);
y_wall = R_cylinder * sin(theta);
x_safe = r_safe * cos(theta);
y_safe = r_safe * sin(theta);

% 绘制圆柱侧面的线 (辅助视觉)
num_lines = 8;
theta_lines = linspace(0, 2*pi, num_lines+1);
theta_lines = theta_lines(1:end-1);
for i = 1:num_lines
    % 物理侧壁 (红)
    x_line_w = R_cylinder * cos(theta_lines(i)) * [1; 1];
    y_line_w = R_cylinder * sin(theta_lines(i)) * [1; 1];
    z_line = [0; H_cylinder];
    plot3(x_line_w, y_line_w, z_line, 'r:', 'LineWidth', 0.5);

    % 安全侧壁 (绿)
    x_line_s = r_safe * cos(theta_lines(i)) * [1; 1];
    y_line_s = r_safe * sin(theta_lines(i)) * [1; 1];
    plot3(x_line_s, y_line_s, z_line, 'g--', 'LineWidth', 1);
end

% 添加坐标轴网格和标签
grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('3D轨迹跟踪: 红色=物理障碍, 绿色=安全边界');
legend('实际轨迹', '参考轨迹', '物理障碍', '物理障碍', '安全边界', 'Location', 'best');

% 设置合适的视角和坐标轴比例
axis equal;
xlim([-5, 5]);
ylim([-5, 5]);
zlim([0, 2]);
view(45, 30);

% 添加额外的子图显示XY平面投影和深度变化
subplot(3,2,3);
plot(X_log(1,:), X_log(2,:), 'b', 'LineWidth', 2); hold on;
plot(ref_traj(1,1:num_steps), ref_traj(2,1:num_steps), 'r--', 'LineWidth', 1.5);
% 添加安全边界（XY平面）
plot(x_wall, y_wall, 'r-', 'LineWidth', 2); % 物理障碍
plot(x_safe, y_safe, 'g--', 'LineWidth', 2); % 安全缓冲
grid on;
xlabel('X (m)');
ylabel('Y (m)');
title('XY平面: 红线=墙, 绿线=缓冲界');
legend('实际', '参考', '物理墙', '缓冲界', 'Location', 'best');
axis equal;

subplot(3,2,4);
time_vec = (0:length(X_log(3,:))-1) * dt;
plot(time_vec, X_log(3,:), 'b', 'LineWidth', 2); hold on;
plot((0:num_steps-1)*dt, ref_traj(3,1:num_steps), 'r--', 'LineWidth', 1.5);
% 添加深度安全边界线
yline(0, 'r-', 'LineWidth', 2); % 物理底
yline(H_cylinder, 'r-', 'LineWidth', 2); % 物理顶
% yline(d_safe, 'g--', 'LineWidth', 2); % 假定深度也有同样的buffer? (如果模型没加则无需画)
grid on;
xlabel('时间 (s)');
ylabel('深度 Z (m)');
title('深度变化: 红色=物理边界');
legend('实际深度', '参考深度', '物理边界', 'Location', 'best');

subplot(3,2,5);
% 显示距离安全边界的余量
r_actual = sqrt(X_log(1,:).^2 + X_log(2,:).^2);
dist_to_wall = R_cylinder - r_actual;
dist_to_buffer = r_safe - r_actual;

% 计算距离点障碍物(h4)的余量
% h4 = (x-Ox)^2 + (y-Oy)^2 - R_safe^2 >= 0
% 物理距离 = sqrt((x-Ox)^2 + (y-Oy)^2) - R_obs
min_dist_to_obs = inf(1, length(time_vec));
for i = 1:size(p_obs_list, 1)
    p_curr = p_obs_list(i,:);
    dist_i = sqrt((X_log(1,:) - p_curr(1)).^2 + (X_log(2,:) - p_curr(2)).^2);
    min_dist_to_obs = min(min_dist_to_obs, dist_i);
end
marg_obs_physical = min_dist_to_obs - (r_safe_obs - 0.2); % 假设物理半径比安全半径小0.2
marg_obs_safety   = min_dist_to_obs - r_safe_obs;         % 对应h4约束

plot(time_vec, dist_to_wall, 'r-', 'LineWidth', 1.0); hold on;
plot(time_vec, dist_to_buffer, 'g--', 'LineWidth', 1.5);
plot(time_vec, marg_obs_physical, 'm-', 'LineWidth', 1.5); % 障碍物物理距离
plot(time_vec, marg_obs_safety, 'm:', 'LineWidth', 1.5);   % 障碍物安全余量

yline(0, 'k-', 'LineWidth', 1);

grid on;
xlabel('时间 (s)');
ylabel('距离 (m)');
title('径向安全余量 (红/绿=墙, 洋红=柱子)');
legend('距物理墙', '距缓冲界', '距柱子实体', '距柱子安全界', '零线', 'Location', 'best');

subplot(3,2,6);
% 显示深度安全余量 (简化显示，仅显示距离物理边界)
depth_margin_lower = X_log(3,:) - 0;
depth_margin_upper = H_cylinder - X_log(3,:);
plot(time_vec, depth_margin_lower, 'b', 'LineWidth', 2); hold on;
plot(time_vec, depth_margin_upper, 'm', 'LineWidth', 2);
yline(0, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('时间 (s)');
ylabel('深度余量 (m)');
title('深度物理余量');
legend('距底面', '距顶面', '边界', 'Location', 'best');

%
% subplot(3,1,2);
% plot(0:dt:T_sim, X_log(3,:));
% title('深度跟踪');
% xlabel('时间(s)'); ylabel('Z位置(m)');
%
% subplot(3,1,3);
% stairs(0:dt:T_sim-dt, U_log');
% title('推进器控制输入');
% xlabel('时间(s)'); ylabel('推力(N)');
