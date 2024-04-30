clc
clear variables
close all

export_images = true;
path = [];

%% Import data

load('mpc_sim_ptc.mat',...
	't', 'prim_res', 'dual_res', 'prim_res_iter', 'dual_res_iter', 'n_iter');
prim_res_ptc = prim_res;
dual_res_ptc = dual_res;
prim_res_iter_ptc = prim_res_iter;
dual_res_iter_ptc = dual_res_iter;

load('mpc_sim_quadprog.mat',...
	'prim_res', 'dual_res');
prim_res_1 = prim_res;
dual_res_1 = dual_res;

load('mpc_sim_mosek.mat',...
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

legend_names = {'PTC','quadprog','MOSEK','OSQP','qpSWIFT','DAQP'};

custom_colors = [0.2 0.2 1;
	1 0.2 0.2;
	1 0.2 1;
	1 0.7 0.2;
	0.2 0.8 0.8;
	0.6 0.3 0.1];

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

hold off, grid on, set(gca,'YMinorGrid','on')
set(gca,'TickLabelInterpreter','latex','fontsize',14)
xlim([0,max_n_iter]), xticks([0,max_n_iter]), xticklabels({'1','$N_\mathrm{iter}$'})
yticks(10.^[-10, -8, -6, -4, -2, 0])
xlabel('RK23 iterations (normalized)','interpreter','latex')
ylabel('Residuals','interpreter','latex')
legend([p2, p1],{'Primal residual','Dual residual'},'interpreter','latex','fontsize',12,...
	'location','southoutside','orientation','horizontal','numcolumns',2)

% - Plot 2: Optimal primal-dual residuals

f2 = figure(2); set(f2,'WindowStyle','normal'); f2.Position = [400   300   480*1.5   300];
tiledlayout(1,2,'tilespacing','tight','padding','tight')

nexttile(1)

p_ptc = semilogy(t(1:end-1), prim_res_ptc, '-', 'linewidth', 1, 'color', custom_colors(1,:)); hold on
p1 = semilogy(t(1:end-1), prim_res_1, '-', 'linewidth', 1, 'color', custom_colors(2,:));
p2 = semilogy(t(1:end-1), prim_res_2, '-', 'linewidth', 1, 'color', custom_colors(3,:));
p3 = semilogy(t(1:end-1), prim_res_3, '-', 'linewidth', 1, 'color', custom_colors(4,:));
p4 = semilogy(t(1:end-1), prim_res_4, '-', 'linewidth', 1, 'color', custom_colors(5,:));
p5 = semilogy(t(1:end-1), prim_res_5, '-', 'linewidth', 1, 'color', custom_colors(6,:));

grid on, grid(gca,'minor')
set(gca,'TickLabelInterpreter','latex','fontsize',14)
xlim([0,t(end)]), xticks(0:1:5)
ylim([10^-16, 10^5]), yticks(10.^[-15, -10, -5, 0, 5])
xlabel('Simulation time [s]','interpreter','latex')
title('Primal residual','interpreter','latex')

nexttile(2)

semilogy(t(1:end-1), dual_res_ptc, '-', 'linewidth', 1, 'color', [custom_colors(1,:), 1]), hold on
semilogy(t(1:end-1), dual_res_1, '-', 'linewidth', 1, 'color', [custom_colors(2,:), 1])
semilogy(t(1:end-1), dual_res_2, '-', 'linewidth', 1, 'color', [custom_colors(3,:), 1])
semilogy(t(1:end-1), dual_res_3, '-', 'linewidth', 1, 'color', [custom_colors(4,:), 1])
semilogy(t(1:end-1), dual_res_4, '-', 'linewidth', 1, 'color', [custom_colors(5,:), 1])
semilogy(t(1:end-1), dual_res_5, '-', 'linewidth', 1, 'color', [custom_colors(6,:), 1])

grid on, grid(gca,'minor')
set(gca,'TickLabelInterpreter','latex','fontsize',14)
xlim([0,t(end)]), xticks(0:1:5)
ylim([10^-16, 10^5]), yticks(10.^[-15, -10, -5, 0, 5])%, yticklabels([])
xlabel('Simulation time [s]','interpreter','latex')
title('Dual residual','interpreter','latex')

ax = nexttile(2);
leg1 = legend(ax,[p_ptc, p1, p2, p3, p4, p5],legend_names,'interpreter','latex','fontsize',12,...
	'location','southoutside','orientation','horizontal','numcolumns',6);
leg1.Layout.Tile = 'South';

if export_images == true

exportgraphics(f1,strcat(path,'sim_prim_dual_res_iter_ptc.pdf'),'BackgroundColor','w','ContentType','vector');
exportgraphics(f2,strcat(path,'sim_prim_dual_res.pdf'),'BackgroundColor','w','ContentType','vector');

end















