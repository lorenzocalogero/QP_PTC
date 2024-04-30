clc
clear variables
close all

export_images = true;
path = [];

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

legend_names = {'PTC','quadprog','Gurobi','MOSEK','OSQP','qpSWIFT','DAQP'};

custom_colors = [0.2 0.2 1;
	1 0.2 0.2;
	0.2 0.8 0.2;
	1 0.2 1;
	1 0.7 0.2;
	0.2 0.8 0.8;
	0.6 0.3 0.1];

%% Plots

% - Plot 1: Execution time (Monte Carlo)

f1 = figure(1); set(f1,'WindowStyle','normal'); f1.Position = [200   200   480   350];
tl1 = tiledlayout(1,1,'tilespacing','none','padding','tight');
colororder(custom_colors(2:end,:))

ylabel(tl1, 'Execution time [ms]','interpreter','latex','fontsize',14)

nexttile, hold on

for i=1:1:N_mc
p1 = scatter(t(1:1:end-1), 1e03*exec_time_mc_1{i}, [], custom_colors(2,:), 'filled', 'MarkerFaceAlpha', 0.65, 'sizedata', 10);
end

for i=1:1:N_mc
p2 = scatter(t(1:1:end-1), 1e03*exec_time_mc_2{i}, [], custom_colors(3,:), 'filled', 'MarkerFaceAlpha', 0.65, 'sizedata', 10);
end

for i=1:1:N_mc
p3 = scatter(t(1:1:end-1), 1e03*exec_time_mc_3{i}, [], custom_colors(4,:), 'filled', 'MarkerFaceAlpha', 0.65, 'sizedata', 10);
end

for i=1:1:N_mc
p4 = scatter(t(1:1:end-1), 1e03*exec_time_mc_4{i}, [], custom_colors(5,:), 'filled', 'MarkerFaceAlpha', 0.65, 'sizedata', 10);
end

for i=1:1:N_mc
p5 = scatter(t(1:1:end-1), 1e03*exec_time_mc_5{i}, [], custom_colors(6,:), 'filled', 'MarkerFaceAlpha', 0.65, 'sizedata', 10);
end

for i=1:1:N_mc
p6 = scatter(t(1:1:end-1), 1e03*exec_time_mc_6{i}, [], custom_colors(7,:), 'filled', 'MarkerFaceAlpha', 0.65, 'sizedata', 10);
end

for i=1:1:N_mc
p_ptc = scatter(t(1:1:end-1), 1e03*exec_time_mc_ptc{i}, [], custom_colors(1,:), 'filled', 'MarkerFaceAlpha', 0.65, 'sizedata', 10);
end

hold off, grid on, grid(gca,'minor')
set(gca,'TickLabelInterpreter','latex','fontsize',14)
ylim([0, 100]), yticks(0:20:100)
xlabel('Simulation time [s]','interpreter','latex')

legend([p_ptc, p1, p2, p3, p4, p5, p6],legend_names,'interpreter','latex','fontsize',11,...
	'location','southoutside','orientation','horizontal','numcolumns',4)

%% Export images

if export_images == true

exportgraphics(f1,strcat(path,'sim_exec_time_mc.pdf'),'BackgroundColor','w','ContentType','vector');

end














