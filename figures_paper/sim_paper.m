clc
clear variables
close all

export_figures = true;
path = '';

%% Import data

load('mpc_sim_ptc.mat',...
	't', 'x_mpc', 'u_mpc', 'x_r_traj', 'x_lb', 'x_ub', 'u_lb', 'u_ub', 'exec_time', 'n_iter');
x_mpc_ptc = x_mpc; u_mpc_ptc = u_mpc; exec_time_ptc = exec_time; n_iter_ptc = n_iter;

load('mpc_sim_quadprog.mat',...
	'x_mpc', 'u_mpc', 'exec_time', 'n_iter');
x_mpc_1 = x_mpc; u_mpc_1 = u_mpc; exec_time_1 = exec_time; n_iter_1 = n_iter;

load('mpc_sim_gurobi.mat',...
	'x_mpc', 'u_mpc', 'exec_time', 'n_iter');
x_mpc_2 = x_mpc; u_mpc_2 = u_mpc; exec_time_2 = exec_time; n_iter_2 = n_iter;

load('mpc_sim_mosek.mat',...
	'x_mpc', 'u_mpc', 'exec_time', 'n_iter');
x_mpc_3 = x_mpc; u_mpc_3 = u_mpc; exec_time_3 = exec_time; n_iter_3 = n_iter;

load('mpc_sim_osqp.mat',...
	'x_mpc', 'u_mpc', 'exec_time', 'n_iter');
x_mpc_4 = x_mpc; u_mpc_4 = u_mpc; exec_time_4 = exec_time; n_iter_4 = n_iter;

load('mpc_sim_qpswift.mat',...
	'x_mpc', 'u_mpc', 'exec_time', 'n_iter');
x_mpc_5 = x_mpc; u_mpc_5 = u_mpc; exec_time_5 = exec_time; n_iter_5 = n_iter;

load('mpc_sim_daqp.mat',...
	'x_mpc', 'u_mpc', 'exec_time', 'n_iter');
x_mpc_6 = x_mpc; u_mpc_6 = u_mpc; exec_time_6 = exec_time; n_iter_6 = n_iter;

load('mpc_sim_piqp.mat',...
	'x_mpc', 'u_mpc', 'exec_time', 'n_iter');
x_mpc_7 = x_mpc; u_mpc_7 = u_mpc; exec_time_7 = exec_time; n_iter_7 = n_iter;

load('mpc_sim_qpalm.mat',...
	'x_mpc', 'u_mpc', 'exec_time', 'n_iter');
x_mpc_8 = x_mpc; u_mpc_8 = u_mpc; exec_time_8 = exec_time; n_iter_8 = n_iter;

load('mpc_sim_true.mat',...
	'x_mpc', 'u_mpc');
x_mpc_true = x_mpc; u_mpc_true = u_mpc;

legend_names = {'PTC','quadprog','Gurobi','MOSEK','OSQP','qpSWIFT','DAQP','PIQP','QPALM'};

custom_colors = [0.2 0.2 1;
	1 0.2 0.2;
	0.2 0.8 0.2;
	1 0.2 1;
	1 0.7 0.2;
	0.2 0.8 0.8;
	0.6 0.3 0.1;
	0.6 0.25 1;
	0.6 1 0.25];

%% Execution time and iterations

fprintf('Solver   | Exec. time:   Max   Mean  | Iter.:   Max   Mean\n')
fprintf('PTC      | %.4f   %.4f | %d  %d\n', max(exec_time_ptc*1e03), mean(exec_time_ptc*1e03), ...
	max(n_iter_ptc), round(mean(n_iter_ptc)))
fprintf('quadprog | %.4f   %.4f | %d  %d\n', max(exec_time_1*1e03), mean(exec_time_1*1e03), ...
	max(n_iter_1), round(mean(n_iter_1)))
fprintf('Gurobi   | %.4f   %.4f | %d  %d\n', max(exec_time_2*1e03), mean(exec_time_2*1e03), ...
	max(n_iter_2), round(mean(n_iter_2)))
fprintf('MOSEK    | %.4f   %.4f | %d  %d\n', max(exec_time_3*1e03), mean(exec_time_3*1e03), ...
	max(n_iter_3), round(mean(n_iter_3)))
fprintf('OSQP     | %.4f   %.4f | %d  %d\n', max(exec_time_4*1e03), mean(exec_time_4*1e03), ...
	max(n_iter_4), round(mean(n_iter_4)))
fprintf('qpSWIFT  | %.4f   %.4f | %d  %d\n', max(exec_time_5*1e03), mean(exec_time_5*1e03), ...
	max(n_iter_5), round(mean(n_iter_5)))
fprintf('DAQP     | %.4f   %.4f | %d  %d\n', max(exec_time_6*1e03), mean(exec_time_6*1e03), ...
	max(n_iter_6), round(mean(n_iter_6)))
fprintf('PIQP     | %.4f   %.4f | %d  %d\n', max(exec_time_7*1e03), mean(exec_time_7*1e03), ...
	max(n_iter_7), round(mean(n_iter_7)))
fprintf('QPALM    | %.4f   %.4f | %d  %d\n', max(exec_time_8*1e03), mean(exec_time_8*1e03), ...
	max(n_iter_8), round(mean(n_iter_8)))

%% Plots

% - Plot 1: 3D trajectory (true vs PTC)

f1 = figure(1); set(f1,'WindowStyle','normal'); f1.Position = [200   200   375   420];
tiledlayout(1,1,'tilespacing','none','padding','tight')

nexttile, hold on

p1 = plot3(x_r_traj(1,:), x_r_traj(2,:), x_r_traj(3,:), 'k-', 'linewidth', 0.5);
p3 = plot3(x_mpc_true(1,:), x_mpc_true(2,:), x_mpc_true(3,:), '-', 'color', custom_colors(2,:), 'linewidth', 1.5);
p2 = plot3(x_mpc_ptc(1,:), x_mpc_ptc(2,:), x_mpc_ptc(3,:), '-', 'color', custom_colors(1,:), 'linewidth', 0.75);
patch([2.5 2.5 2.5 2.5], [-0.5 2.5 2.5 -0.5], [0 0 4 4], 'k','facealpha', 0.2)

hold off, grid on, axis equal
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([-1.5,3.5]), ylim([-0.5,2.5]), zlim([0, 4])
xlabel('$x$ [m]','interpreter','latex')
ylabel('$y$ [m]','interpreter','latex')
zlabel('$z$ [m]','interpreter','latex')
view(-35,50)
leg1 = legend([p3, p2, p1],{'Optimal','PTC','Reference'},'interpreter','latex','fontsize',11,...
	'location','southoutside','orientation','horizontal','numcolumns',3);
leg1.ItemTokenSize(1) = 15;
title('(a)','interpreter','latex','fontsize',14)

% - Plot 2: Orientation (true vs PTC)

f2 = figure(2); set(f2,'WindowStyle','normal'); f2.Position = [400   300   375   340];
tl1 = tiledlayout(3,1,'tilespacing','none','padding','tight');
title(tl1,'(b)','interpreter','latex','fontsize',14)

% -- 1:
nexttile(1), hold on

p3 = yline(rad2deg(x_lb(4)), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1]);
yline(rad2deg(x_ub(4)), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1])
p2 = plot(t, rad2deg(x_mpc_true(4,:)), '-', 'color', custom_colors(2,:), 'linewidth', 1.5);
p1 = plot(t, rad2deg(x_mpc_ptc(4,:)), '-', 'color', custom_colors(1,:), 'linewidth', 0.75);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,t(end)]), ylim([1.1*rad2deg(x_lb(4)),1.1*rad2deg(x_ub(4))])
xticklabels([])
ylabel('$\phi$ [deg]','interpreter','latex')

% -- 2:
nexttile(2), hold on

p3 = yline(rad2deg(x_lb(5)), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1]);
yline(rad2deg(x_ub(5)), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1])
p2 = plot(t, rad2deg(x_mpc_true(5,:)), '-', 'color', custom_colors(2,:), 'linewidth', 1.5);
p1 = plot(t, rad2deg(x_mpc_ptc(5,:)), '-', 'color', custom_colors(1,:), 'linewidth', 0.75);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,t(end)]), ylim([1.1*rad2deg(x_lb(5)),1.1*rad2deg(x_ub(5))])
xticklabels([])
ylabel('$\theta$ [deg]','interpreter','latex')

% -- 3:
nexttile(3), hold on

p3 = yline(rad2deg(x_lb(6)), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1]);
yline(rad2deg(x_ub(6)), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1])
p2 = plot(t, rad2deg(x_mpc_true(6,:)), '-', 'color', custom_colors(2,:), 'linewidth', 1.5);
p1 = plot(t, rad2deg(x_mpc_ptc(6,:)), '-', 'color', custom_colors(1,:), 'linewidth', 0.75);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,t(end)]), ylim([1.1*rad2deg(x_lb(6)),1.1*rad2deg(x_ub(6))])
xlabel('Simulation time [s]','interpreter','latex')
ylabel('$\psi$ [deg]','interpreter','latex')

ax = nexttile(3);
leg1 = legend(ax,[p2, p1, p3],{'Optimal','PTC','Bounds'},'interpreter','latex','fontsize',11,...
	'location','southoutside','orientation','horizontal','numcolumns',3);
leg1.Layout.Tile = 'South';
leg1.ItemTokenSize(1) = 15;

% - Plot 3: Control inputs (true vs PTC)

f3 = figure(3); set(f3,'WindowStyle','normal'); f3.Position = [600   200   440   340];
tl1 = tiledlayout(2,2,'tilespacing','compact','padding','tight');
title(tl1,'(c)','interpreter','latex','fontsize',14)

% -- 1:
nexttile, hold on

p3 = yline(u_lb(1), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1]);
yline(u_ub(1), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1])
p2 = plot(t(1:1:end-1), u_mpc_true(1,:), '-', 'color', custom_colors(2,:), 'linewidth', 1.5);
p1 = plot(t(1:1:end-1), u_mpc_ptc(1,:), '-', 'color', custom_colors(1,:), 'linewidth', 0.75);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,t(end)]), ylim([-1,1.1*u_ub(1)])
xticks(0:5), xticklabels(0:5)
ylabel('$f$ [N]','interpreter','latex')

% -- 2:
nexttile, hold on

p3 = yline(u_lb(2), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1]);
yline(u_ub(2), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1])
p2 = plot(t(1:1:end-1), u_mpc_true(2,:), '-', 'color', custom_colors(2,:), 'linewidth', 1.5);
p1 = plot(t(1:1:end-1), u_mpc_ptc(2,:), '-', 'color', custom_colors(1,:), 'linewidth', 0.75);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,5]), ylim([1.1*u_lb(2),1.1*u_ub(2)])
xticks(0:5), xticklabels(0:5)
ylabel('$\tau_x$ [$\mathrm{N \, m}$]','interpreter','latex')

% -- 3:
nexttile, hold on

p3 = yline(u_lb(3), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1]);
yline(u_ub(3), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1])
p2 = plot(t(1:1:end-1), u_mpc_true(3,:), '-', 'color', custom_colors(2,:), 'linewidth', 1.5);
p1 = plot(t(1:1:end-1), u_mpc_ptc(3,:), '-', 'color', custom_colors(1,:), 'linewidth', 0.75);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,t(end)]), ylim([1.1*u_lb(3),1.1*u_ub(3)])
xticks(0:5), xticklabels(0:5)
xlabel('Simulation time [s]','interpreter','latex')
ylabel('$\tau_y$ [$\mathrm{N \, m}$]','interpreter','latex')

% -- 4:
nexttile, hold on

p3 = yline(u_lb(4), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1]);
yline(u_ub(4), '--', 'linewidth', 0.75, 'color', [0.1 0.1 0.1])
p2 = plot(t(1:1:end-1), u_mpc_true(4,:), '-', 'color', custom_colors(2,:), 'linewidth', 1.5);
p1 = plot(t(1:1:end-1), u_mpc_ptc(4,:), '-', 'color', custom_colors(1,:), 'linewidth', 0.75);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,t(end)]), ylim([1.1*u_lb(4),1.1*u_ub(4)])
xticks(0:5), xticklabels(0:5)
xlabel('Simulation time [s]','interpreter','latex')
ylabel('$\tau_z$ [$\mathrm{N \, m}$]','interpreter','latex')

ax = nexttile(4);
leg1 = legend(ax,[p2, p1, p3],{'Optimal','PTC','Bounds'},'interpreter','latex','fontsize',11,...
	'location','southoutside','orientation','horizontal','numcolumns',3);
leg1.Layout.Tile = 'South';
leg1.ItemTokenSize(1) = 15;

% - Plot 4: Execution time

f4 = figure(4); set(f4,'WindowStyle','normal'); f4.Position = [800   300   480   420];
tl1 = tiledlayout(3,1,'tilespacing','tight','padding','tight');
colororder(custom_colors(2:end,:))

ylabel(tl1,'Execution time [ms]','interpreter','latex','fontsize',13)

nexttile([2 1]), hold on

p1 = plot(t(1:1:end-1), 1e03*exec_time_1, '.-', 'linewidth', 1, 'markersize', 7);
p2 = plot(t(1:1:end-1), 1e03*exec_time_2, '.-', 'linewidth', 1, 'markersize', 7);
p3 = plot(t(1:1:end-1), 1e03*exec_time_3, '.-', 'linewidth', 1, 'markersize', 7);
p4 = plot(t(1:1:end-1), 1e03*exec_time_4, '.-', 'linewidth', 1, 'markersize', 7);
p5 = plot(t(1:1:end-1), 1e03*exec_time_5, '.-', 'linewidth', 1, 'markersize', 7);
p6 = plot(t(1:1:end-1), 1e03*exec_time_6, '.-', 'linewidth', 1, 'markersize', 7);
p7 = plot(t(1:1:end-1), 1e03*exec_time_7, '.-', 'linewidth', 1, 'markersize', 7);
p8 = plot(t(1:1:end-1), 1e03*exec_time_8, '.-', 'linewidth', 1, 'markersize', 7);
p_ptc = plot(t(1:1:end-1), 1e03*exec_time_ptc, '.-', 'color', custom_colors(1,:), 'linewidth', 1, 'markersize', 7);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xticks(0:5), xticklabels([])
ylim([0, 100]), yticks(0:20:100)
title('(a)','interpreter','latex','fontsize',14)

nexttile(3), hold on

p1 = plot(t(1:1:end-1), 1e03*exec_time_1, '.-', 'linewidth', 1, 'markersize', 7);
p2 = plot(t(1:1:end-1), 1e03*exec_time_2, '.-', 'linewidth', 1, 'markersize', 7);
p3 = plot(t(1:1:end-1), 1e03*exec_time_3, '.-', 'linewidth', 1, 'markersize', 7);
p4 = plot(t(1:1:end-1), 1e03*exec_time_4, '.-', 'linewidth', 1, 'markersize', 7);
p5 = plot(t(1:1:end-1), 1e03*exec_time_5, '.-', 'linewidth', 1, 'markersize', 7);
p6 = plot(t(1:1:end-1), 1e03*exec_time_6, '.-', 'linewidth', 1, 'markersize', 7);
p7 = plot(t(1:1:end-1), 1e03*exec_time_7, '.-', 'linewidth', 1, 'markersize', 7);
p8 = plot(t(1:1:end-1), 1e03*exec_time_8, '.-', 'linewidth', 1, 'markersize', 7);
p_ptc = plot(t(1:1:end-1), 1e03*exec_time_ptc, '.-', 'color', custom_colors(1,:), 'linewidth', 1, 'markersize', 7);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
ylim([0 5])
xlabel('Simulation time [s]','interpreter','latex')
xticks(0:5), xticklabels(0:5)
title('(b)','interpreter','latex','fontsize',14)

leg1 = legend([p_ptc, p1, p2, p3, p4, p5, p6, p7, p8],legend_names,'interpreter','latex','fontsize',11,...
	'location','southoutside','orientation','horizontal','numcolumns',5);
leg1.ItemTokenSize(1) = 15;

%% Export images

if export_figures == true

exportgraphics(f1,strcat(path,'sim_ptc_1.pdf'),'BackgroundColor','w','ContentType','vector');
exportgraphics(f2,strcat(path,'sim_ptc_2.pdf'),'BackgroundColor','w','ContentType','vector');
exportgraphics(f3,strcat(path,'sim_ptc_3.pdf'),'BackgroundColor','w','ContentType','vector');
exportgraphics(f4,strcat(path,'sim_exec_time.pdf'),'BackgroundColor','w','ContentType','vector');

end














