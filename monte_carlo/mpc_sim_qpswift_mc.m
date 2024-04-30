clc
clear variables
close all

disp_curr_exec_time = true;
save_data = true;
save_name = 'mpc_sim_qpswift_mc_1.mat';
path = '../sim_data/monte_carlo/';

%% Plant data

nx = 12;
nu = 4;

Ts = 20e-03; % Discrete time step
T_int = 1e-03; % Plant model integration time step

%% MPC data

% Horizons
Np = 40;
Nc = 10;

Tp = 20e-03; % Prediction time step

% Weighting matrices
Q = diag([1e02, 1e02, 1e03, 1e-03, 1e-03, 1e02, 1e-03, 1e-03, 1e-03, 1e-03, 1e-03, 1e-03]);
R = diag([1e-04, 1e-04, 1e-04, 1e-04]);
Rd = diag([1e01, 1e01, 1e01, 1e01]);
P = 1e01*Q;
S = 1e06*eye(nx);

% Bounds
u_lb = [0, -2, -2, -2]';
u_ub = [15, 2, 2, 2]';

x_lb = [-1.5, -0.5, 0, deg2rad(-75), deg2rad(-75), deg2rad(-180), -inf, -inf, -inf, -inf, -inf, -inf]';
x_ub = [2.5, 2.5, 4, deg2rad(75), deg2rad(75), deg2rad(180), inf, inf, inf, inf, inf, inf]';

%% MPC formulation

% ===== Cost function =====

% Q_bar

cell_tmp = repmat({Q},Np-1,1); cell_tmp{Np} = P;
Q_bar = blkdiag(cell_tmp{:});

% R_bar, T

cell_tmp = repmat({R},Np,1);
R_bar_tilde = blkdiag(cell_tmp{:});

mat_tmp_1 = repmat(eye(nu),floor(Np/Nc),1);
mat_tmp_2 = repmat(eye(nu),floor(Np/Nc)+mod(Np,Nc),1);
cell_tmp = [repmat({mat_tmp_1},Nc-1,1); {mat_tmp_2}];
T = blkdiag(cell_tmp{:});

R_bar = T'*R_bar_tilde*T;

% R_delta_bar

mat_tmp = 2*eye(Np) + diag(-1*ones(Np-1,1),1) + diag(-1*ones(Np-1,1),-1);
mat_tmp(1,1) = 1; mat_tmp(Np,Np) = 1;
Rd_bar_tilde = kron(mat_tmp,Rd);

Rd_bar = T'*Rd_bar_tilde*T;

% H, f

H = blkdiag(Q_bar, R_bar + Rd_bar, S);

c = @(X_r) [-Q_bar'*X_r; zeros(nu*Nc,1); zeros(nx,1)];

% ===== Equality constraints (prediction model) =====

% Ap
mat_tmp = diag(ones(Np-1,1),-1);
Ap = @(A) kron(mat_tmp,A);

% Bp
mat_tmp = diag(ones(Np,1));
Bp = @(B) kron(mat_tmp,B)*T;

% cp
cp = @(x0, A, c) [A*x0 + c; repmat(c,Np-1,1)];

% A_eq, b_eq
cell_tmp = repmat({eye(nx)},Np,1);
A_eq = @(A, B) [blkdiag(cell_tmp{:}) - Ap(A), -Bp(B), zeros(nx*Np,nx)];
b_eq = @(x0, A, c) cp(x0, A, c);

% ===== Inequality constraints (bounds) =====

% Input
Eu = [eye(nu*Nc); -eye(nu*Nc)];
hu = [repmat(u_ub,Nc,1); repmat(-u_lb,Nc,1)];

% Slack
Ee = -eye(nx);
he = zeros(nx,1);

% State
Ex = [eye(nx*Np); -eye(nx*Np)];
Te = repmat(eye(nx),Np,1); Eep = [Te; Te];
hx = [repmat(x_ub,floor(0.5*Np),1); repmat(inf*ones(nx,1),Np-floor(0.5*Np),1);...
	repmat(-x_lb,floor(0.5*Np),1); repmat(inf*ones(nx,1),Np-floor(0.5*Np),1)];

% A_ineq, B_ineq
A_ineq = [Ex, zeros(2*nx*Np, nu*Nc), -Eep;
	zeros(2*nu*Nc, nx*Np), Eu, zeros(2*nu*Nc,nx);
	zeros(nx, nx*Np), zeros(nx, nu*Nc), Ee];
b_ineq = [hx; hu; he];

% Remove unbounded ineq. constraints
ind_inf = (b_ineq == inf);
b_ineq(ind_inf) = []; A_ineq(ind_inf,:) = [];

%% MPC simulation

T = 5;
N = floor(T/Ts);
N_sim = N;
t = linspace(0, N_sim*Ts, N_sim+1);

N_mc = 50; % N. of Monte Carlo simulations

% Reference traj. (lemniscate)
traj = @(t) [1 + 2*cos(sqrt(2)*2*pi*t);
	1 + 2*sin(sqrt(2)*2*pi*t).*cos(sqrt(2)*2*pi*t);
	2 + 0*t;
	0*t;
	0*t;
	pi/4 + 0*t;
	zeros(nx-6,1)*t];
x_r_traj = traj((0:Ts:T)/T);

x_mpc = zeros(nx,N_sim+1);
u_mpc = zeros(nu,N_sim);

% Initial states (MC)

x0_lb = [x_lb(1), x_lb(2), x_lb(3), 0, 0, pi/4, 0, 0, 0, 0, 0, 0]';
x0_ub = [x_ub(1), x_ub(2), x_ub(3), 0, 0, pi/4, 0, 0, 0, 0, 0, 0]';

x0 = x0_lb + (x0_ub - x0_lb).*rand(nx,N_mc);

u0 = zeros(nu,1); % Initial input guess

% ===== MPC simulation start (Monte Carlo) =====
exec_time_mc = cell(N_mc,1);

for n=1:1:N_mc

exec_time = []; n_iter = [];

x_mpc(:,1) = x0(:,n);

fprintf('MPC simulation (qpSWIFT) (Monte Carlo) (%d / %d)...\n', n, N_mc);
for k=1:1:N_sim
	
	% Reference traj. over pred. horizon
	if N - k >= Np - 1
		X_r = x_r_traj(:,k:1:k+Np-1);
		X_r = reshape(X_r, [nx*Np, 1]);
	else
		vec_tmp_1 = x_r_traj(:,k:1:end);
		vec_tmp_2 = repmat(x_r_traj(:,end), 1, Np - size(x_r_traj(:,k:1:end),2));
		X_r = [vec_tmp_1, vec_tmp_2];
		X_r = reshape(X_r, [nx*Np, 1]);
	end
	
	% Linearize pred. model
	if k == 1
		J_f_x = num_jacobian(@(x)plant_dt(x,u0,Tp), x_mpc(:,k));
		J_f_u = num_jacobian(@(u)plant_dt(x_mpc(:,k),u,Tp), u0);
		
		A = J_f_x;
		B = J_f_u;
		b = plant_dt(x_mpc(:,k),u0,Tp) - A*x_mpc(:,k) - B*u0;
	else
		J_f_x = num_jacobian(@(x)plant_dt(x,u_mpc(:,k-1),Tp), x_mpc(:,k));
		J_f_u = num_jacobian(@(u)plant_dt(x_mpc(:,k),u,Tp), u_mpc(:,k-1));
		
		A = J_f_x;
		B = J_f_u;
		b = plant_dt(x_mpc(:,k),u_mpc(:,k-1),Tp) - A*x_mpc(:,k) - B*u_mpc(:,k-1);
	end

	% QP-MPC matrices
	H_mpc = H;
	c_mpc = c(X_r);
	A_eq_mpc = A_eq(A,B);
	b_eq_mpc = b_eq(x_mpc(:,k), A, b);
	A_ineq_mpc = A_ineq;
	b_ineq_mpc = b_ineq;

	% Primal-dual infeas. functions
	prim_inf_fun = @(y) max( norm(A_eq_mpc*y-b_eq_mpc,inf), norm(max(A_ineq_mpc*y-b_ineq_mpc,0),inf) ); % Primal infeas.
	dual_inf_fun = @(y,mu,lambda) norm(H_mpc*y + c_mpc - A_eq_mpc'*mu - A_ineq_mpc'*lambda,inf); % Dual infeas.

	options.abstol = 1e-06; % Infeas. abs. tolerance
	options.reltol = 1e-06; % Infeas. rel. tolerance

	tim1 = tic;
	
	[y_opt, basic_info, adv_info] = qpSWIFT(sparse(H_mpc), c_mpc, sparse(A_eq_mpc), b_eq_mpc, ...
		sparse(A_ineq_mpc), b_ineq_mpc, options);
	
	curr_exec_time = toc(tim1);
	curr_n_iter = basic_info.Iterations;

	exec_time = [exec_time, curr_exec_time];
	n_iter = [n_iter, curr_n_iter];
	
	if disp_curr_exec_time == true
		fprintf('MPC step solved (%d / %d) (time: %.4f ms, iterations: %d)\n',...
			k, N, curr_exec_time*1e03, curr_n_iter)
	end

	x_opt = y_opt(1:1:nx*Np);
	x_opt = reshape(x_opt, [nx,Np]);
	
	u_opt = y_opt(nx*Np+1:1:nx*Np+nu*Nc);
	u_opt = reshape(u_opt, [nu,Nc]);
	
	e_opt = y_opt(nx*Np+nu*Nc+1:1:end);
	
	u_mpc(:,k) = u_opt(:,1);

	% Plant model integration
	x_tmp = x_mpc(:,k);
	for i = 1:1:floor(Ts/T_int)
		x_tmp = plant_dt(x_tmp,u_mpc(:,k),T_int);
	end
	x_mpc(:,k+1) = x_tmp;
	
end

exec_time_mc{n} = exec_time;

end

if save_data == true

save(strcat(path,save_name), 't', 'exec_time_mc');

end

%% Plots

% - Plot 1: Execution time (MC)

f1 = figure(1); set(f1,'WindowStyle','docked');
tiledlayout(1,1,'tilespacing','none','padding','tight')

nexttile, hold on

for i=1:1:N_mc
plot(t(1:1:end-1), 1e03*exec_time_mc{i}, 'r.', 'markersize', 10);
end

hold off, grid on, grid(gca,'minor')
set(gca,'TickLabelInterpreter','latex','fontsize',14)
xlabel('Simulation time [s]','interpreter','latex')
ylabel('Execution time [ms]','interpreter','latex')

% ========================================

function xs = sat(x,bounds)
xs = min(max(x,bounds(:,1)),bounds(:,2));
end







