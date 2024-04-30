function xd0 = plant_ct(x0,u0)

% x = [x, y, z, phi, theta, psi, vx, vy, vz, v_phi, v_theta, v_psi]
% u = [f, tau_x, tau_y, tau_z]

x = x0(1);
y = x0(2);
z = x0(3);
phi = x0(4);
theta = x0(5);
psi = x0(6);
vx = x0(7);
vy = x0(8);
vz = x0(9);
v_phi = x0(10);
v_theta = x0(11);
v_psi = x0(12);

f = u0(1);
tau_x = u0(2);
tau_y = u0(3);
tau_z = u0(4);

m = 0.5;
Ix = 5e-03;
Iy = 5e-03;
Iz = 2e-02;
g = 9.81;
beta = 0.25;

x_dot = vx;
y_dot = vy;
z_dot = vz;
phi_dot = v_phi;
theta_dot = v_theta;
psi_dot = v_psi;
vx_dot = (1 / m) * (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * f - (beta / m) * vx;
vy_dot = (1 / m) * (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * f - (beta / m) * vy;
vz_dot = (1 / m) * (cos(phi) * cos(theta)) * f - g - (beta / m) * vz;
v_phi_dot = (Iy - Iz) / Ix * v_theta * v_psi + tau_x / Ix;
v_theta_dot = (Iz - Ix) / Iy * v_phi * v_psi + tau_y / Iy;
v_psi_dot = (Ix - Iy) / Iz * v_theta * v_phi + tau_z / Iz;

xd0 = [x_dot, y_dot, z_dot, phi_dot, theta_dot, psi_dot,...
	vx_dot, vy_dot, vz_dot, v_phi_dot, v_theta_dot, v_psi_dot]';

end