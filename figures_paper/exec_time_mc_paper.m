clc
clear variables
close all

export_figures = true;
path = '';

%% Import data

load('mpc_sim_ptc_mc.mat',...
	't', 'exec_time_mc');
exec_time_mc_ptc = exec_time_mc;
N_mc = length(exec_time_mc_ptc);

load('mpc_sim_quadprog_mc.mat',...
	'exec_time_mc');
exec_time_mc_1 = exec_time_mc;

load('mpc_sim_gurobi_mc.mat',...
	'exec_time_mc');
exec_time_mc_2 = exec_time_mc;

load('mpc_sim_mosek_mc.mat',...
	'exec_time_mc');
exec_time_mc_3 = exec_time_mc;

load('mpc_sim_osqp_mc.mat',...
	'exec_time_mc');
exec_time_mc_4 = exec_time_mc;

load('mpc_sim_qpswift_mc.mat',...
	'exec_time_mc');
exec_time_mc_5 = exec_time_mc;

load('mpc_sim_daqp_mc.mat',...
	'exec_time_mc');
exec_time_mc_6 = exec_time_mc;

load('mpc_sim_piqp_mc.mat',...
	'exec_time_mc');
exec_time_mc_7 = exec_time_mc;

load('mpc_sim_qpalm_mc.mat',...
	'exec_time_mc');
exec_time_mc_8 = exec_time_mc;

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

%% Plots

% - Plot 1: Execution time (Monte Carlo)

f1 = figure(1); set(f1,'WindowStyle','normal'); f1.Position = [200   200   480   350];
tl1 = tiledlayout(1,1,'tilespacing','none','padding','tight');
colororder(custom_colors(2:end,:))

ylabel(tl1,'Execution time [ms]','interpreter','latex','fontsize',13)

nexttile, hold on

mark_size = 8;
mark_alpha = 0.6;

for i=1:1:N_mc
p1 = scatter(t(1:1:end-1), 1e03*exec_time_mc_1{i}, [], custom_colors(2,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p2 = scatter(t(1:1:end-1), 1e03*exec_time_mc_2{i}, [], custom_colors(3,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p3 = scatter(t(1:1:end-1), 1e03*exec_time_mc_3{i}, [], custom_colors(4,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p4 = scatter(t(1:1:end-1), 1e03*exec_time_mc_4{i}, [], custom_colors(5,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p5 = scatter(t(1:1:end-1), 1e03*exec_time_mc_5{i}, [], custom_colors(6,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p6 = scatter(t(1:1:end-1), 1e03*exec_time_mc_6{i}, [], custom_colors(7,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p7 = scatter(t(1:1:end-1), 1e03*exec_time_mc_7{i}, [], custom_colors(8,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p8 = scatter(t(1:1:end-1), 1e03*exec_time_mc_8{i}, [], custom_colors(9,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p_ptc = scatter(t(1:1:end-1), 1e03*exec_time_mc_ptc{i}, [], custom_colors(1,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
ylim([0, 100]), yticks(0:20:100)
xlabel('Simulation time [s]','interpreter','latex')
leg1 = legend([p_ptc, p1, p2, p3, p4, p5, p6, p7, p8],legend_names,'interpreter','latex','fontsize',11,...
	'location','southoutside','orientation','horizontal','numcolumns',5);
leg1.ItemTokenSize(1) = 15;

% - Plot 2: Execution time (Monte Carlo, with detail)

f2 = figure(2); set(f2,'WindowStyle','normal'); f2.Position = [400   200   480   420];
tl1 = tiledlayout(3,1,'tilespacing','tight','padding','tight');
colororder(custom_colors(2:end,:))

ylabel(tl1,'Execution time [ms]','interpreter','latex','fontsize',13)

nexttile([2 1]), hold on

for i=1:1:N_mc
p1 = scatter(t(1:1:end-1), 1e03*exec_time_mc_1{i}, [], custom_colors(2,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p2 = scatter(t(1:1:end-1), 1e03*exec_time_mc_2{i}, [], custom_colors(3,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p3 = scatter(t(1:1:end-1), 1e03*exec_time_mc_3{i}, [], custom_colors(4,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p4 = scatter(t(1:1:end-1), 1e03*exec_time_mc_4{i}, [], custom_colors(5,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p5 = scatter(t(1:1:end-1), 1e03*exec_time_mc_5{i}, [], custom_colors(6,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p6 = scatter(t(1:1:end-1), 1e03*exec_time_mc_6{i}, [], custom_colors(7,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p7 = scatter(t(1:1:end-1), 1e03*exec_time_mc_7{i}, [], custom_colors(8,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p8 = scatter(t(1:1:end-1), 1e03*exec_time_mc_8{i}, [], custom_colors(9,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p_ptc = scatter(t(1:1:end-1), 1e03*exec_time_mc_ptc{i}, [], custom_colors(1,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
xticks(0:5), xticklabels([])
ylim([0, 100]), yticks(0:20:100)
title('(a)','interpreter','latex','fontsize',14)

nexttile(3), hold on

for i=1:1:N_mc
p1 = scatter(t(1:1:end-1), 1e03*exec_time_mc_1{i}, [], custom_colors(2,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p2 = scatter(t(1:1:end-1), 1e03*exec_time_mc_2{i}, [], custom_colors(3,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p3 = scatter(t(1:1:end-1), 1e03*exec_time_mc_3{i}, [], custom_colors(4,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p4 = scatter(t(1:1:end-1), 1e03*exec_time_mc_4{i}, [], custom_colors(5,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p5 = scatter(t(1:1:end-1), 1e03*exec_time_mc_5{i}, [], custom_colors(6,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p6 = scatter(t(1:1:end-1), 1e03*exec_time_mc_6{i}, [], custom_colors(7,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p7 = scatter(t(1:1:end-1), 1e03*exec_time_mc_7{i}, [], custom_colors(8,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p8 = scatter(t(1:1:end-1), 1e03*exec_time_mc_8{i}, [], custom_colors(9,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

for i=1:1:N_mc
p_ptc = scatter(t(1:1:end-1), 1e03*exec_time_mc_ptc{i}, [], custom_colors(1,:), 'filled', 'MarkerFaceAlpha', mark_alpha, 'sizedata', mark_size);
end

hold off, grid on, grid(gca,'minor'), box on
set(gca,'TickLabelInterpreter','latex','fontsize',13)
ylim([0 5])
xlabel('Simulation time [s]','interpreter','latex')
xticks(0:5), xticklabels(0:5)
title('(b)','interpreter','latex','fontsize',14)

leg1 = legend([p_ptc, p1, p2, p3, p4, p5, p6, p7, p8],legend_names,'interpreter','latex','fontsize',11,...
	'location','southoutside','orientation','horizontal','numcolumns',5);
leg1.ItemTokenSize(1) = 15;

%% Export figures

if export_figures == true

exportgraphics(f2,strcat(path,'sim_exec_time_mc.pdf'),'BackgroundColor','w','ContentType','vector');

end














