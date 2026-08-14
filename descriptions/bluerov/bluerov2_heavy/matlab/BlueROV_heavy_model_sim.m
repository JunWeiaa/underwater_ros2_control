function model = BlueROV_heavy_model_sim()
import casadi.*
nx = 13;
nu = 8;
ref_path = SX.sym('ref_path',nx);
thrust_health = SX.sym('thrust_health',nu);  % 推进器健康状态（8维）
p_param = vertcat(ref_path, thrust_health);
%% 定义状态变量（13维：位置+四元数+速度）
p = SX.sym('p',3);
q = SX.sym('q',4);
v = SX.sym('v',3);
w = SX.sym('w',3);
x = vertcat(p,q,v,w);   % [X,Y,Z, q0,q1,q2,q3, u,v,w, p,q,r]
u = SX.sym('u', nu);    % 推进器控制输入
xdot = SX.sym('xdot',nx);
%% 模型参数（与原始模型一致）
m = 13.0;             % 质量 (kg)
zg = -0.011;            % 重心z坐标 (m)
xg=0.0;
zb = -0.06;            % 浮心z坐标 (m)
B = 141.635725;            % 浮力 (N)
W = 127.4;            % 重量 (N)
Xu = -5.5;            % 流体动力系数
Yv = -12.7;
Zw = -14.57;
Kp = -0.12;
Mq = -0.12;
Nr = -0.12;
Im = [0.26, 0.23, 0.37];  % 惯性矩
D_l = [-4.03, -6.22, -5.18, -0.07, -0.07, -0.07];
D_q = [-18.18, -21.66, -36.99, -1.55, -1.55, -1.55];

% %% 推进器配置矩阵（保持不变）
% l = 0.001 * [         % 推进器位置 (m)
%     156,  111, 85;
%     156, -111, 85;
%     -156,  111, 85;
%     -156, -111, 85;
%     120,  218, 0;
%     120, -218, 0;
%     -120,  218, 0;
%     -120, -218, 0;
%     ];

% % 构建推力分配矩阵T
% T_T = zeros(8,6);
% angles = [pi/4, -pi/4, 3*pi/4, -3*pi/4];
% for i = 1:4
%     theta = angles(i);
%     T_T(i,:) = [cos(theta), -sin(theta), 0,...
%         sin(theta)*l(i,3), cos(theta)*l(i,3),...
%         -sin(theta)*l(i,1)-cos(theta)*l(i,2)];
% end
% for i = 5:8
%     % 确定推进器方向
%     if i == 6 ||i==7
%         thrust_dir = -1;  %
%     else
%         thrust_dir = 1;   %
%     end

%     % 构建推力矩阵行
%     T_T(i,:) = [0, 0, -thrust_dir,...
%         -l(i,2)*thrust_dir,...
%         l(i,1)*thrust_dir,...
%         0];
% end
% T = T_T'
%% 推进器分配矩阵 (直接定义)
% T矩阵: 6x8 矩阵，将8个推进器推力映射到6DOF力和力矩
% 行: [Fx, Fy, Fz, Mx, My, Mz] (FRD坐标系)
% 列: 推进器1-8
T = [
    0.7071    0.7071   -0.7071   -0.7071         0         0         0         0;
    -0.7071    0.7071   -0.7071    0.7071         0         0         0         0;
    0         0         0         0   -1.0000    1.0000    1.0000   -1.0000;
    0.0601   -0.0601    0.0601   -0.0601   -0.2180   -0.2180    0.2180    0.2180;
    0.0601    0.0601   -0.0601   -0.0601    0.1200   -0.1200    0.1200   -0.1200;
    -0.1888    0.1888    0.1888   -0.1888         0         0         0         0
    ];
%% 四元数处理
q_no = q / norm(q);  % 归一化四元数
% x(4:7) =q;
q0 = q_no(1); q1 = q_no(2); q2 = q_no(3); q3 = q_no(4);

%% 坐标系变换
% % 四元数到旋转矩阵(FLU → ENU)
% R_b2n = [1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
%     2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
%     2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),  1-2*(q1^2+q2^2)];
% 四元数到旋转矩阵 (FRD → NED)
R_b2n = [1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
    2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
    2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),  1-2*(q1^2+q2^2)];
% 角速度转换矩阵（四元数微分）
T_q2n = 0.5 * [-q1, -q2, -q3;
    q0, -q3,  q2;
    q3,  q0, -q1;
    -q2,  q1,  q0];



%% 动力学参数计算
% 科氏力和向心力矩阵
u_b = v(1); v_b = v(2); w_b = v(3);
p_b = w(1); q_b = w(2); r_b = w(3);

CRB = [0,    0,    0,    0,    m*w_b, -m*v_b;
    0,    0,    0,    -m*w_b, 0,    m*u_b;
    0,    0,    0,    m*v_b, -m*u_b, 0;
    0,    m*w_b, -m*v_b, 0,    Im(3)*r_b, -Im(2)*q_b;
    -m*w_b, 0,    m*u_b, -Im(3)*r_b, 0,    Im(1)*p_b;
    m*v_b, -m*u_b, 0,    Im(2)*q_b, -Im(1)*p_b, 0];

CA = [0,    0,    0,    0,    Zw*w_b, 0;
    0,    0,    0,    -Zw*w_b, 0,    -Xu*u_b;
    0,    0,    0,    Yv*v_b, Xu*u_b, 0;
    0,    Zw*w_b, -Yv*v_b, 0,    -Nr*r_b, Mq*q_b;
    Zw*w_b, 0,    Xu*u_b, Nr*r_b, 0,    -Kp*p_b;
    -Yv*v_b, Xu*u_b, 0,    -Mq*q_b, Kp*p_b, 0];

% 质量矩阵
MRB = [m, 0, 0, 0, m*zg, 0;
    0, m, 0, -m*zg, 0, 0;
    0, 0, m, 0, 0, 0;
    0, -m*zg, 0, Im(1), 0, 0;
    m*zg, 0, 0, 0, Im(2), 0;
    0, 0, 0, 0, 0, Im(3)];
MA = -diag([Xu, Yv, Zw, Kp, Mq, Nr]);

% 阻尼矩阵
D = -(diag(D_l) + diag(D_q.*abs([u_b, v_b, w_b, p_b, q_b, r_b])));

%% 恢复力计算 (正确的物理建模)
Z = p(3);  % 当前深度 (正值表示在水下)
B_adj = if_else(Z > 0, B, 0.0);  % 只有在水下才有浮力

% 重力和浮力始终在导航系(NED)中作用
gravity_n = [0; 0; W];        % 重力：向下(+Z)
buoyancy_n = [0; 0; -B_adj];  % 浮力：向上(-Z)

% 转换到机体坐标系 (用于动力学方程)
gravity_b = -R_b2n' * gravity_n;   % 重力在机体系中的表示
buoyancy_b = -R_b2n' * buoyancy_n; % 浮力在机体系中的表示

% 合力 (机体坐标系)
force_restoring = gravity_b + buoyancy_b;

% 恢复力矩计算 (机体坐标系)
% 力矩 = r × F (叉积)，其中 r = [0; 0; z]
% 对于恢复力矩，我们使用：M = -r × F (负号表示恢复特性)
% 叉积：[0; 0; z] × [Fx; Fy; Fz] = [z*Fy; -z*Fx; 0]
% 恢复力矩：-[z*Fy; -z*Fx; 0] = [-z*Fy; z*Fx; 0]s
moment_gravity = [zg * gravity_b(2);   % -r_cg(3) * gravity_b(2)
    -xg*gravity_b(3)+zg * gravity_b(1);   %  r_cg(3) * gravity_b(1)
    xg*gravity_b(2)];                  %  0

moment_buoyancy = [zb * buoyancy_b(2); % -r_cb(3) * buoyancy_b(2)
    -zb * buoyancy_b(1); %  r_cb(3) * buoyancy_b(1)
    0];                 %  0


moment_restoring = moment_gravity + moment_buoyancy;

% 恢复力向量 [Fx; Fy; Fz; Mx; My; Mz] (机体坐标系)
g = [force_restoring; moment_restoring];

%% 动力学方程
forces = T*u - (CRB + CA)*[v;w] - g - D*[v;w];
accel_b = (MRB + MA) \ forces;
a_b =  accel_b(1:3);
omega_dot = accel_b(4:6);
% 转换到导航系
v_n = R_b2n * v;
%% 状态导数
f_expl_expr = vertcat(v_n, ...         % 位置导数
    T_q2n*w, ...      % 四元数导数
    a_b, ...          % 机体系加速度
    omega_dot);       % 角加速度

f_impl_expr = f_expl_expr - xdot;
%% cost in nonlinear least squares form

Q_pos = diag([80,80, 200]);       % 位置误差权重
Q_att = diag([250, 250, 300]);    % 姿态误差虚部权重
Q_vel = diag([1, 1, 1]); % 速度误差权重
Q_ang = diag([0.5, 0.5, 0.5]);% 角速度误差权重
% Q_pos = diag([50, 50, 50]);       % 位置误差权重
% Q_att = diag([10, 10, 10]);    % 姿态误差虚部权重
% Q_vel = diag([1, 1, 1, 1, 1, 1]); % 速度误差权重
Q_fpos = 10*Q_pos;       % 位置误差权重
Q_fatt = 10*Q_att;    % 姿态误差虚部权重
Q_fvel = 10*Q_vel; % 速度误差权重
Q_fang = 10*Q_ang; % 角速度误差权重
R = diag(1*ones(8,1));          % 控制输入权重
%% 定义四元数乘法匿名函数 (Hamilton乘积规则)
quatmultiply = @(q1, q2) [...
    q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);  % 实部
    q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);  % i分量
    q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);  % j分量
    q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1)   % k分量
    ];

%% 在代价函数中使用（示例）
q_ref = ref_path(4:7);  % 参考姿态（需先归一化）
q_current = q/norm(q);  % 当前姿态

% 计算相对旋转误差
q_error = quatmultiply(q_ref, [q_current(1); -q_current(2:4)]);  % q_ref ⊗ q⁻¹
att_error =2* q_error(2:4);  % 提取虚部
% 1-N cost term
cost_expr_ext_cost_0 = ...
    0.5*(x(1:3)-ref_path(1:3))'*Q_pos*(x(1:3)-ref_path(1:3)) + ...
    0.5*(att_error)'*Q_att*(att_error) + ...
    0.5*(v-ref_path(8:10))'*Q_vel*(v-ref_path(8:10)) + ...
    0.5*(w-ref_path(11:13))'*Q_ang*(w-ref_path(11:13)) + ...
    0.5*u'*R*u;
cost_expr_ext_cost = ...
    0.5*(x(1:3)-ref_path(1:3))'*Q_pos*(x(1:3)-ref_path(1:3)) + ...
    0.5*(att_error)'*Q_att*(att_error) + ...
    0.5*(v-ref_path(8:10))'*Q_vel*(v-ref_path(8:10)) + ...
    0.5*(w-ref_path(11:13))'*Q_ang*(w-ref_path(11:13))+ ...
    0.5*u'*R*u;

% final cost term

%% 终端代价单独定义（如果需要不同权重）
cost_expr_ext_cost_e = ...  % 终端代价表达式
    0.5*(x(1:3)-ref_path(1:3))'*Q_fpos*(x(1:3)-ref_path(1:3)) + ...
    0.5*(att_error)'*Q_fatt*(att_error) + ...
    0.5*(v-ref_path(8:10))'*Q_fvel*(v-ref_path(8:10))+ ...
    0.5*(w-ref_path(11:13))'*Q_fang*(w-ref_path(11:13));
cost_expr_ext_cost_custom_hess = blkdiag(R,Q_pos,0,Q_att,Q_vel,Q_ang);
cost_expr_ext_cost_custom_hess_e = blkdiag(Q_fpos,0,Q_fatt,Q_fvel,Q_fang);


con_h_expr_e = [norm(x(1:3)-ref_path(1:3),inf);
    norm(att_error(1:2),inf);
    ];

%% HCBF 约束 (圆柱桶安全区域)
R_cylinder = 4.0; % 圆柱半径 (m)
H_cylinder = 1.6; % 圆柱高度 (m)
d_safe = 1.0;     % 增大安全距离到0.3m，提供更多缓冲
d_depth = 0;
% HCBF 增益参数 (确保多项式 s^2 + K1*s + K0 是Hurwitz的)
% 对于水下机器人，需要考虑其相对较慢的动态响应
% 较小的增益确保约束不会过于激进，避免数值问题

% 径向约束：相对保守，允许较缓慢的趋近边界
K_radial = [20, 30];  % [K0, K1] - 较温和的响应

% 深度约束：稍微激进一些，因为深度控制通常更关键
K_lower = [15, 20];    % [K0, K1] - 下边界稍微严格
K_upper = [15, 20];    % [K0, K1] - 上边界稍微严格

% 安全函数定义
% h1: 径向约束 h1 = R^2 - (x^2 + y^2) >= 0
h1 = (R_cylinder - d_safe)^2 - (p(1)^2 + p(2)^2);

% h2: 深度下限约束 h2 = z - z_min >= 0 (确保 z >= d_safe)
h2 = p(3) - d_depth;

% h3: 深度上限约束 h3 = z_max - z >= 0 (确保 z <= H_cylinder - d_safe)
h3 = (H_cylinder - d_depth) - p(3);

% 计算一阶Lie导数 L_f h_j
% L_f h1 = ∇h1^T * ṗ = ∇h1^T * R * v
grad_h1 = [-2*p(1); -2*p(2); 0];  % ∇h1
grad_h2 = [0; 0; 1];               % ∇h2
grad_h3 = [0; 0; -1];              % ∇h3

L_f_h1 = grad_h1' * R_b2n * v;
L_f_h2 = grad_h2' * R_b2n * v;
L_f_h3 = grad_h3' * R_b2n * v;

% 计算二阶Lie导数 L_f^2 h_j
% L_f^2 h_j = ∇h_j^T * (S(ω)R*v + R*M^(-1)*τ_drift)
% 其中 S(ω) 是反对称矩阵, τ_drift = -C*ν - D*ν - g
tau_drift = -(CRB + CA)*[v;w] - g - D*[v;w];

% 反对称矩阵 S(ω)
S_omega = [0, -w(3), w(2);
    w(3), 0, -w(1);
    -w(2), w(1), 0];

% 计算 L_f^2 h_j (只需要力的部分，前3个元素)
M_inv = (MRB + MA) \ tau_drift;
rdot_term = S_omega * R_b2n * v + R_b2n * M_inv(1:3);
L_f2_h1 = grad_h1' * rdot_term;
L_f2_h2 = grad_h2' * rdot_term;
L_f2_h3 = grad_h3' * rdot_term;

% 计算混合Lie导数 L_g L_f h_j
% L_g L_f h_j = ∇h_j^T * R * M^(-1) * T
M_inv_T = (MRB + MA) \ T*diag(thrust_health);
L_g_Lf_h1 = grad_h1' * R_b2n * M_inv_T(1:3, :);
L_g_Lf_h2 = grad_h2' * R_b2n * M_inv_T(1:3, :);
L_g_Lf_h3 = grad_h3' * R_b2n * M_inv_T(1:3, :);

% HCBF 约束: L_f^2 h + L_g L_f h * u + k^T * ψ >= 0
% 其中 ψ = [h, L_f h]^T, k = [K0, K1]^T
% 重新排列为: L_g L_f h * u >= -L_f^2 h - K1*L_f h - K0*h

% 约束右侧项
b1 = -L_f2_h1 - K_radial(2)*L_f_h1 - K_radial(1)*h1;
b2 = -L_f2_h2 - K_lower(2)*L_f_h2 - K_lower(1)*h2;
b3 = -L_f2_h3 - K_upper(2)*L_f_h3 - K_upper(1)*h3;

% HCBF 约束矩阵形式: A_j * u >= b_j
% 在 Acados 中，约束形式为 C*u - d >= 0，即 u 的线性约束
% 所以我们定义为: L_g_Lf_h * u - b >= 0

% 径向HCBF约束
con_h_radial = L_g_Lf_h1 * u - b1;

% 深度下限HCBF约束
con_h_lower = L_g_Lf_h2 * u - b2;

% 深度上限HCBF约束
con_h_upper = L_g_Lf_h3 * u - b3;

% 合并所有HCBF约束
con_h_expr = vertcat(con_h_radial, con_h_lower, con_h_upper);

% 暂时注释掉完整的HCBF约束
% con_h_expr = vertcat(con_h_radial, con_h_lower, con_h_upper);
%% 构建模型
model = AcadosModel();
model.x = x;
model.u = u;
model.xdot=xdot;
model.p = p_param;
model.f_expl_expr = f_expl_expr;
model.f_impl_expr = f_impl_expr;
model.cost_expr_ext_cost_0 = cost_expr_ext_cost_0;
model.cost_expr_ext_cost = cost_expr_ext_cost;
model.cost_expr_ext_cost_e = cost_expr_ext_cost_e;
model.name = 'BlueROV_Heavy_sim';
% model.cost_expr_ext_cost_custom_hess= cost_expr_ext_cost_custom_hess;
% model.cost_expr_ext_cost_custom_hess_e= cost_expr_ext_cost_custom_hess_e;
model.con_h_expr = con_h_expr;
% model.con_h_expr_e = con_h_expr_e;
end
