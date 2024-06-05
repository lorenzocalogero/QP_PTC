clc
clear variables
close all

export_figures = true;
path = '';

%% Import data

load('mpc_sim_ptc.mat',...
	't', 'prim_res', 'dual_res', 'prim_res_iter', 'dual_res_iter', 'n_iter');
prim_res_ptc = prim_res;
dual_res_ptc = dual_res;
prim_res_iter_ptc = prim_res_iter;
dual_res_iter_ptc = dual_res_iter;

load('mpc_sim_quadprog_res.mat',...
	'prim_res', 'dual_res');
prim_res_1 = prim_res;
dual_res_1 = dual_res;

load('mpc_sim_mosek_res.mat',...
	'prim_res', 'dual_res');
prim_res_2 = prim_res;
dual_res_2 = dual_res;

load('mpc_sim_osqp.mat',...
	'prim_res', 'dual_res');
prim_res_3 = prim_res;
dual_res_3 = dual_res;

load('mpc_sim_qpswift.mat',...
	'prim_res', 'dual_res');
prim_res_4 = prim_res;
dual_res_4 = dual_res;

load('mpc_sim_daqp.mat',...
	'prim_res', 'dual_res');
prim_res_5 = prim_res;
dual_res_5 = dual_res;

load('mpc_sim_piqp.mat',...
	'prim_res', 'dual_res');
prim_res_6 = prim_res;
dual_res_6 = dual_res;

load('mpc_sim_qpalm.mat',...
	'prim_res', 'dual_res');
prim_res_7 = prim_res;
dual_res_7 = dual_res;

legend_names = {'PTC','quadprog','MOSEK','OSQP','qpSWIFT','DAQP','PIQP','QPALM'};

custom_colors = [0.2 0.2 1;
	1 0.2 0.2;
	1 0.2 1;
	1 0.7 0.2;
	0.2 0.8 0.8;
	0.6 0.3 0.1;
	0.6 0.25 1;
	0.6 1 0.25];

%% Plots

% - Plot 1: Primal-dual residuals (PTC) for every RK2(3) iteration

f1 = figure(1); set(f1,'WindowStyle','normal'); f1.Position = [200   200   420   300];
tiledlayout(1,1,'tilespacing','none','padding','tight')

nexttile

max_n_iter = max(n_iter);

for k=1:1:length(t)-1
	curr_n_iter = length(prim_res_iter_ptc{k});
	if curr_n_iter > 1
		x_axis = (0:1:curr_n_iter-1)*(max_n_iter/(curr_n_iter-1));
		p1 = semilogy(x_axis, dual_res_iter_ptc{k}, '-', 'linewidth', 2, 'color', [1, 0.2, 0.2, 0.75]); hold on
	end
end

for k=1:1:length(t)-1
	curr_n_iter = length(prim_res_iter_ptc{k});
	if curr_n_iter > 1
		x_axis = (0:1:curr_n_iter-1)*(max_n_iter/(curr_n_iter-1));
		p2 = semilogy(x_axis, prim_res_iter_ptc{k}, '-', 'linewidth', 2, 'color', [0.2, 0.2, 1, 0.75]); hold on
	end
end

hold off, grid on, set(gca,'YMinorGrid','on'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,max_n_iter]), xticks([0,max_n_iter]), xticklabels({'start','end'})
yticks(10.^[-10, -8, -6, -4, -2, 0])
xlabel('Iterations (normalized)','interpreter','latex')
ylabel('Residuals','interpreter','latex')
leg1 = legend([p2, p1],{'Primal residual','Dual residual'},'interpreter','latex','fontsize',11,...
	'location','southoutside','orientation','horizontal','numcolumns',2);
leg1.ItemTokenSize(1) = 15;

% - Plot 2: Optimal primal-dual residuals

f2 = figure(2); set(f2,'WindowStyle','normal'); f2.Position = [400   300   425   390];
tiledlayout(2,1,'tilespacing','tight','padding','tight')

nexttile(1)

mark_size = 10;

p_ptc = semilogy(t(1:end-1), prim_res_ptc, '.', 'markersize', mark_size, 'color', custom_colors(1,:)); hold on
p1 = semilogy(t(1:end-1), prim_res_1, '.', 'markersize', mark_size, 'color', custom_colors(2,:));
p2 = semilogy(t(1:end-1), prim_res_2, '.', 'markersize', mark_size, 'color', custom_colors(3,:));
p3 = semilogy(t(1:end-1), prim_res_3, '.', 'markersize', mark_size, 'color', custom_colors(4,:));
p4 = semilogy(t(1:end-1), prim_res_4, '.', 'markersize', mark_size, 'color', custom_colors(5,:));
p5 = semilogy(t(1:end-1), prim_res_5, '.', 'markersize', mark_size, 'color', custom_colors(6,:));
p6 = semilogy(t(1:end-1), prim_res_6, '.', 'markersize', mark_size, 'color', custom_colors(7,:));
p7 = semilogy(t(1:end-1), prim_res_7, '.', 'markersize', mark_size, 'color', custom_colors(8,:));

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,t(end)]), xticks(0:1:5), xticklabels([])
ylim([10^-16, 10^-4]), yticks(10.^[-15, -10, -5])
%xlabel('Simulation time [s]','interpreter','latex')
title('Primal residual','interpreter','latex','fontsize',14)

nexttile(2)

semilogy(t(1:end-1), dual_res_ptc, '.', 'markersize', mark_size, 'color', [custom_colors(1,:), 1]), hold on
semilogy(t(1:end-1), dual_res_1, '.', 'markersize', mark_size, 'color', [custom_colors(2,:), 1])
semilogy(t(1:end-1), dual_res_2, '.', 'markersize', mark_size, 'color', [custom_colors(3,:), 1])
semilogy(t(1:end-1), dual_res_3, '.', 'markersize', mark_size, 'color', [custom_colors(4,:), 1])
semilogy(t(1:end-1), dual_res_4, '.', 'markersize', mark_size, 'color', [custom_colors(5,:), 1])
semilogy(t(1:end-1), dual_res_5, '.', 'markersize', mark_size, 'color', [custom_colors(6,:), 1])
semilogy(t(1:end-1), dual_res_6, '.', 'markersize', mark_size, 'color', [custom_colors(7,:), 1])
semilogy(t(1:end-1), dual_res_7, '.', 'markersize', mark_size, 'color', [custom_colors(8,:), 1])

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xlim([0,t(end)]), xticks(0:1:5)
ylim([10^-16, 10^0]), yticks(10.^[-15, -10, -5, 0])
xlabel('Simulation time [s]','interpreter','latex')
title('Dual residual','interpreter','latex','fontsize',14)

ax = nexttile(2);
leg1 = legend(ax,[p_ptc, p1, p2, p3, p4, p5, p6, p7],legend_names,'interpreter','latex','fontsize',12,...
	'location','southoutside','orientation','horizontal','numcolumns',4);
leg1.Layout.Tile = 'South';
leg1.ItemTokenSize(1) = 15;

% - Plot 3: Optimal primal-dual residuals (scatter)

f3 = figure(3); set(f3,'WindowStyle','normal'); f3.Position = [600   200   425   390];
tiledlayout(2,1,'tilespacing','tight','padding','tight')

nexttile(1)

mark_size = 8;
mark_alpha = 0.65;

hold on

p_ptc = scatter(t(1:1:end-1), prim_res_ptc, [], custom_colors(1,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p1 = scatter(t(1:1:end-1), prim_res_1, [], custom_colors(2,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p2 = scatter(t(1:1:end-1), prim_res_2, [], custom_colors(3,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p3 = scatter(t(1:1:end-1), prim_res_3, [], custom_colors(4,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p4 = scatter(t(1:1:end-1), prim_res_4, [], custom_colors(5,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p5 = scatter(t(1:1:end-1), prim_res_5, [], custom_colors(6,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p6 = scatter(t(1:1:end-1), prim_res_6, [], custom_colors(7,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p7 = scatter(t(1:1:end-1), prim_res_7, [], custom_colors(8,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13,'YScale','log')
xlim([0,t(end)]), xticks(0:1:5), xticklabels([])
ylim([10^-16, 10^-4]), yticks(10.^[-15, -10, -5])
%xlabel('Simulation time [s]','interpreter','latex')
title('Primal residual','interpreter','latex','fontsize',14)

nexttile(2)

hold on

p_ptc = scatter(t(1:1:end-1), dual_res_ptc, [], custom_colors(1,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p1 = scatter(t(1:1:end-1), dual_res_1, [], custom_colors(2,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p2 = scatter(t(1:1:end-1), dual_res_2, [], custom_colors(3,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p3 = scatter(t(1:1:end-1), dual_res_3, [], custom_colors(4,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p4 = scatter(t(1:1:end-1), dual_res_4, [], custom_colors(5,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p5 = scatter(t(1:1:end-1), dual_res_5, [], custom_colors(6,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p6 = scatter(t(1:1:end-1), dual_res_6, [], custom_colors(7,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
p7 = scatter(t(1:1:end-1), dual_res_7, [], custom_colors(8,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13,'YScale','log')
xlim([0,t(end)]), xticks(0:1:5)
ylim([10^-16, 10^0]), yticks(10.^[-15, -10, -5, 0])
xlabel('Simulation time [s]','interpreter','latex')
title('Dual residual','interpreter','latex','fontsize',14)

ax = nexttile(2);
leg1 = legend(ax,[p_ptc, p1, p2, p3, p4, p5, p6, p7],legend_names,'interpreter','latex','fontsize',12,...
	'location','southoutside','orientation','horizontal','numcolumns',4);
leg1.Layout.Tile = 'South';
leg1.ItemTokenSize(1) = 15;

%% Export figures

if export_figures == true

exportgraphics(f1,strcat(path,'sim_prim_dual_res_iter_ptc.pdf'),'BackgroundColor','w','ContentType','vector');
% exportgraphics(f2,strcat(path,'sim_prim_dual_res.pdf'),'BackgroundColor','w','ContentType','vector');
exportgraphics(f3,strcat(path,'sim_prim_dual_res.pdf'),'BackgroundColor','w','ContentType','vector');

end















