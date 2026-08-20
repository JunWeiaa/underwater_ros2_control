function model = BlueROV_heavy_model_sim()
import casadi.*
nx = 13;
nu = 8;
ref_path = SX.sym('ref_path',nx);
thrust_health = SX.sym('thrust_health',nu);  % Thruster health state
p_param = vertcat(ref_path, thrust_health);
%% State variables: position, quaternion, linear velocity, angular velocity
p = SX.sym('p',3);
q = SX.sym('q',4);
v = SX.sym('v',3);
w = SX.sym('w',3);
x = vertcat(p,q,v,w);   % [X,Y,Z, q0,q1,q2,q3, u,v,w, p,q,r]
u = SX.sym('u', nu);    % Thruster control input
xdot = SX.sym('xdot',nx);
%% Model parameters
m = 13.0;               % Mass (kg)
zg = -0.011;            % Center of gravity z coordinate (m)
xg=0.0;
zb = -0.06;             % Center of buoyancy z coordinate (m)
B = 141.635725;         % Buoyancy (N)
W = 127.4;              % Weight (N)
Xu = -5.5;              % Hydrodynamic coefficient
Yv = -12.7;
Zw = -14.57;
Kp = -0.12;
Mq = -0.12;
Nr = -0.12;
Im = [0.26, 0.23, 0.37];  % Moments of inertia
D_l = [-4.03, -6.22, -5.18, -0.07, -0.07, -0.07];
D_q = [-18.18, -21.66, -36.99, -1.55, -1.55, -1.55];

%% Thruster allocation matrix
% 6x8 matrix mapping 8 thruster forces to 6-DOF force and moment.
% Rows: [Fx, Fy, Fz, Mx, My, Mz] in the FRD frame.
% Columns: thrusters 1-8.
T = [
    0.7071    0.7071   -0.7071   -0.7071         0         0         0         0;
    -0.7071    0.7071   -0.7071    0.7071         0         0         0         0;
    0         0         0         0   -1.0000    1.0000    1.0000   -1.0000;
    0.0601   -0.0601    0.0601   -0.0601   -0.2180   -0.2180    0.2180    0.2180;
    0.0601    0.0601   -0.0601   -0.0601    0.1200   -0.1200    0.1200   -0.1200;
    -0.1888    0.1888    0.1888   -0.1888         0         0         0         0
    ];
%% Quaternion handling
q_no = q / norm(q);  % Normalized quaternion
% x(4:7) =q;
q0 = q_no(1); q1 = q_no(2); q2 = q_no(3); q3 = q_no(4);

%% Frame transforms
% Quaternion to rotation matrix (FRD to NED).
R_b2n = [1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3),   2*(q1*q3 + q0*q2);
    2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),     2*(q2*q3 - q0*q1);
    2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1),  1-2*(q1^2+q2^2)];
% Angular-velocity matrix for quaternion derivative.
T_q2n = 0.5 * [-q1, -q2, -q3;
    q0, -q3,  q2;
    q3,  q0, -q1;
    -q2,  q1,  q0];



%% Dynamics terms
% Coriolis and centripetal matrices.
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

% Mass matrix.
MRB = [m, 0, 0, 0, m*zg, 0;
    0, m, 0, -m*zg, 0, 0;
    0, 0, m, 0, 0, 0;
    0, -m*zg, 0, Im(1), 0, 0;
    m*zg, 0, 0, 0, Im(2), 0;
    0, 0, 0, 0, 0, Im(3)];
MA = -diag([Xu, Yv, Zw, Kp, Mq, Nr]);

% Damping matrix.
D = -(diag(D_l) + diag(D_q.*abs([u_b, v_b, w_b, p_b, q_b, r_b])));

%% Restoring force model
Z = p(3);  % Current depth; positive is underwater
B_adj = if_else(Z > 0, B, 0.0);  % Apply buoyancy only underwater

% Gravity and buoyancy act in the NED navigation frame.
gravity_n = [0; 0; W];        % Gravity: down (+Z)
buoyancy_n = [0; 0; -B_adj];  % Buoyancy: up (-Z)

% Convert to the body frame for dynamics.
gravity_b = -R_b2n' * gravity_n;
buoyancy_b = -R_b2n' * buoyancy_n;

force_restoring = gravity_b + buoyancy_b;

% Restoring moments in the body frame, using M = -r x F.
moment_gravity = [zg * gravity_b(2);   % -r_cg(3) * gravity_b(2)
    -xg*gravity_b(3)+zg * gravity_b(1);   %  r_cg(3) * gravity_b(1)
    xg*gravity_b(2)];                  %  0

moment_buoyancy = [zb * buoyancy_b(2); % -r_cb(3) * buoyancy_b(2)
    -zb * buoyancy_b(1); %  r_cb(3) * buoyancy_b(1)
    0];                 %  0


moment_restoring = moment_gravity + moment_buoyancy;

% Restoring vector [Fx; Fy; Fz; Mx; My; Mz] in the body frame.
g = [force_restoring; moment_restoring];

%% Dynamics
forces = T*u - (CRB + CA)*[v;w] - g - D*[v;w];
accel_b = (MRB + MA) \ forces;
a_b =  accel_b(1:3);
omega_dot = accel_b(4:6);
v_n = R_b2n * v;
%% State derivatives
f_expl_expr = vertcat(v_n, ...         % Position derivative
    T_q2n*w, ...      % Quaternion derivative
    a_b, ...          % Body-frame acceleration
    omega_dot);       % Angular acceleration

f_impl_expr = f_expl_expr - xdot;
%% cost in nonlinear least squares form

Q_pos = diag([80,80, 200]);      % Position error weight
Q_att = diag([250, 250, 300]);   % Attitude imaginary-part error weight
Q_vel = diag([1, 1, 1]);         % Linear velocity error weight
Q_ang = diag([0.5, 0.5, 0.5]);   % Angular velocity error weight
Q_fpos = 10*Q_pos;               % Terminal position error weight
Q_fatt = 10*Q_att;               % Terminal attitude error weight
Q_fvel = 10*Q_vel;               % Terminal linear velocity error weight
Q_fang = 10*Q_ang;               % Terminal angular velocity error weight
R = diag(1*ones(8,1));           % Control input weight
%% Quaternion multiplication helper, Hamilton product
quatmultiply = @(q1, q2) [...
    q1(1)*q2(1) - q1(2)*q2(2) - q1(3)*q2(3) - q1(4)*q2(4);  % Scalar part
    q1(1)*q2(2) + q1(2)*q2(1) + q1(3)*q2(4) - q1(4)*q2(3);  % i component
    q1(1)*q2(3) - q1(2)*q2(4) + q1(3)*q2(1) + q1(4)*q2(2);  % j component
    q1(1)*q2(4) + q1(2)*q2(3) - q1(3)*q2(2) + q1(4)*q2(1)   % k component
    ];

%% Attitude error for the cost function
q_ref = ref_path(4:7);  % Reference attitude, expected normalized
q_current = q/norm(q);  % Current attitude

q_error = quatmultiply(q_ref, [q_current(1); -q_current(2:4)]);  % q_ref * q^{-1}
att_error =2* q_error(2:4);  % Imaginary part
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

%% Terminal cost
cost_expr_ext_cost_e = ...
    0.5*(x(1:3)-ref_path(1:3))'*Q_fpos*(x(1:3)-ref_path(1:3)) + ...
    0.5*(att_error)'*Q_fatt*(att_error) + ...
    0.5*(v-ref_path(8:10))'*Q_fvel*(v-ref_path(8:10))+ ...
    0.5*(w-ref_path(11:13))'*Q_fang*(w-ref_path(11:13));
cost_expr_ext_cost_custom_hess = blkdiag(R,Q_pos,0,Q_att,Q_vel,Q_ang);
cost_expr_ext_cost_custom_hess_e = blkdiag(Q_fpos,0,Q_fatt,Q_fvel,Q_fang);


con_h_expr_e = [norm(x(1:3)-ref_path(1:3),inf);
    norm(att_error(1:2),inf);
    ];

%% HCBF constraints for a cylindrical safety region
R_cylinder = 4.0; % Cylinder radius (m)
H_cylinder = 1.6; % Cylinder height (m)
d_safe = 1.0;     % Safety margin (m)
d_depth = 0;
% HCBF gains chosen so s^2 + K1*s + K0 is Hurwitz.
% Underwater vehicles respond slowly, so moderate gains avoid aggressive
% constraints and numerical issues.

K_radial = [20, 30];  % Conservative radial response

K_lower = [15, 20];   % Lower-depth boundary response
K_upper = [15, 20];   % Upper-depth boundary response

% Safety functions.
% h1: radial constraint, h1 = R^2 - (x^2 + y^2) >= 0
h1 = (R_cylinder - d_safe)^2 - (p(1)^2 + p(2)^2);

% h2: lower depth constraint, h2 = z - z_min >= 0
h2 = p(3) - d_depth;

% h3: upper depth constraint, h3 = z_max - z >= 0
h3 = (H_cylinder - d_depth) - p(3);

% First Lie derivatives, L_f h_j.
% L_f h1 = grad(h1)' * pdot = grad(h1)' * R * v
grad_h1 = [-2*p(1); -2*p(2); 0];  % grad(h1)
grad_h2 = [0; 0; 1];               % grad(h2)
grad_h3 = [0; 0; -1];              % grad(h3)

L_f_h1 = grad_h1' * R_b2n * v;
L_f_h2 = grad_h2' * R_b2n * v;
L_f_h3 = grad_h3' * R_b2n * v;

% Second Lie derivatives.
% L_f^2 h_j = grad(h_j)' * (S(w)R*v + R*M^(-1)*tau_drift)
% S(w) is skew-symmetric, tau_drift = -C*nu - D*nu - g.
tau_drift = -(CRB + CA)*[v;w] - g - D*[v;w];

% Skew-symmetric angular-rate matrix.
S_omega = [0, -w(3), w(2);
    w(3), 0, -w(1);
    -w(2), w(1), 0];

% L_f^2 h_j uses only the force terms, the first 3 elements.
M_inv = (MRB + MA) \ tau_drift;
rdot_term = S_omega * R_b2n * v + R_b2n * M_inv(1:3);
L_f2_h1 = grad_h1' * rdot_term;
L_f2_h2 = grad_h2' * rdot_term;
L_f2_h3 = grad_h3' * rdot_term;

% Mixed Lie derivative L_g L_f h_j = grad(h_j)' * R * M^(-1) * T.
M_inv_T = (MRB + MA) \ T*diag(thrust_health);
L_g_Lf_h1 = grad_h1' * R_b2n * M_inv_T(1:3, :);
L_g_Lf_h2 = grad_h2' * R_b2n * M_inv_T(1:3, :);
L_g_Lf_h3 = grad_h3' * R_b2n * M_inv_T(1:3, :);

% HCBF constraint:
% L_f^2 h + L_g L_f h * u + k' * psi >= 0,
% where psi = [h, L_f h]' and k = [K0, K1]'.

b1 = -L_f2_h1 - K_radial(2)*L_f_h1 - K_radial(1)*h1;
b2 = -L_f2_h2 - K_lower(2)*L_f_h2 - K_lower(1)*h2;
b3 = -L_f2_h3 - K_upper(2)*L_f_h3 - K_upper(1)*h3;

% Acados uses C*u - d >= 0, so define L_g_Lf_h*u - b >= 0.
con_h_radial = L_g_Lf_h1 * u - b1;

con_h_lower = L_g_Lf_h2 * u - b2;

con_h_upper = L_g_Lf_h3 * u - b3;

con_h_expr = vertcat(con_h_radial, con_h_lower, con_h_upper);

%% Build model
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
model.con_h_expr = con_h_expr;
end
