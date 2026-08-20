function model = subcat_model()
import casadi.*
nx = 17;
nu = 8;
ref_path = SX.sym('ref_path',13);
thrust_health = SX.sym('thrust_health',nu);  % Thruster health state
%% State variables: position, quaternion, velocity, and servo angles
p = SX.sym('p',3);
q = SX.sym('q',4);
v = SX.sym('v',3);
w = SX.sym('w',3);
theta = SX.sym('theta',4); % Servo angles
x = vertcat(p,q,v,w,theta);   % [X,Y,Z, q0,q1,q2,q3, u,v,w, p,q,r, theta1..theta4]
thr = SX.sym('thr',4);
theta_dot = SX.sym('theta_dot',4); % Servo angular rates
u = vertcat(thr,theta_dot);    % Thruster and servo control input
xdot = SX.sym('xdot',nx);
%% Model parameters
m = 13.25;             % Mass (kg)
zg = -0.00205876;      % Center of gravity z coordinate (m)
zb = -0.03;            % Center of buoyancy z coordinate (m)
xg = -0.00634838;
B = 143.0;             % Buoyancy (N)
W = 129.85;            % Weight (N)
Xu = -8.7;             % Added mass coefficient
Yv = -11.32;
Zw = -20.68;
Kp = -0.142;
Mq = -0.268;
Nr = -0.195;
Im = [0.23337, 0.301, 0.4981];  % Moments of inertia
% Paper magnitudes are stored with negative signs to match the model convention.
D_l = [-0.387, -1.315, 0, -0.139, 0, -0.0854];
D_q = [-30.379, -33.525, -117.412, -0.504, -0.602, -0.487];

%% Quaternion handling
q_no = q / norm(q);  % Normalized quaternion
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
lx1 = 0.18775;
lx2 = 0.18275;
ly = 0.260159;
lz=0.020148429;
% Restoring vector [Fx; Fy; Fz; Mx; My; Mz] in the body frame.
g = [force_restoring; moment_restoring];
tau = [ -sin(theta(1))*thr(1)-sin(theta(2))*thr(2)-sin(theta(3))*thr(3)-sin(theta(4))*thr(4);
    0;
    cos(theta(1))*thr(1)+cos(theta(2))*thr(2)+cos(theta(3))*thr(3)+cos(theta(4))*thr(4);
    ly*(cos(theta(1))*thr(1)+cos(theta(2))*thr(2)-cos(theta(3))*thr(3)-cos(theta(4))*thr(4));
    -cos(theta(1))*thr(1)*lx1+cos(theta(2))*thr(2)*lx2+cos(theta(3))*thr(3)*lx2-cos(theta(4))*thr(4)*lx1+sin(theta(1))*thr(1)*lz+sin(theta(2))*thr(2)*lz+sin(theta(3))*thr(3)*lz+sin(theta(4))*thr(4)*lz;
    ly*(sin(theta(1))*thr(1)+sin(theta(2))*thr(2)-sin(theta(3))*thr(3)-sin(theta(4))*thr(4))];
%% Dynamics
forces = tau - (CRB + CA)*[v;w] - g - D*[v;w];
accel_b = (MRB + MA) \ forces;
a_b =  accel_b(1:3);
omega_dot = accel_b(4:6);
v_n = R_b2n * v;
%% State derivatives
f_expl_expr = vertcat(  v_n, ...         % Position derivative
    T_q2n*w, ...      % Quaternion derivative
    a_b, ...          % Body-frame acceleration
    omega_dot,...         % Angular acceleration
    theta_dot);

f_impl_expr = f_expl_expr - xdot;
%% cost in nonlinear least squares form

Q_pos = diag([80,80, 200]);       % Position error weight
Q_att = diag([250, 250, 300]);    % Attitude imaginary-part error weight
Q_vel = diag([1, 1, 1]);          % Linear velocity error weight
Q_ang = diag([1, 1, 1]);          % Angular velocity error weight
Q_fpos = 10*Q_pos;                % Terminal position error weight
Q_fatt = 10*Q_att;                % Terminal attitude error weight
Q_fvel = 10*Q_vel;                % Terminal linear velocity error weight
Q_fang = 10*Q_ang;                % Terminal angular velocity error weight
R = diag([1,1,1,1,0.1,0.1,0.1,0.1]);          % Control input weight
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
att_error = q_error(2:4);  % Imaginary part
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

%% Terminal cost
cost_expr_ext_cost_e = ...
    0.5*(x(1:3)-ref_path(1:3))'*Q_fpos*(x(1:3)-ref_path(1:3)) + ...
    0.5*(att_error)'*Q_fatt*(att_error) + ...
    0.5*(v-ref_path(8:10))'*Q_fvel*(v-ref_path(8:10))+ ...
    0.5*(w-ref_path(11:13))'*Q_fang*(w-ref_path(11:13));

%% Build model
model = AcadosModel();
model.x = x;
model.u = u;
model.xdot=xdot;
model.p = ref_path;
model.f_expl_expr = f_expl_expr;
model.f_impl_expr = f_impl_expr;
model.cost_expr_ext_cost_0 = cost_expr_ext_cost_0;
model.cost_expr_ext_cost = cost_expr_ext_cost;
model.cost_expr_ext_cost_e = cost_expr_ext_cost_e;
model.name = 'subcat';
end
