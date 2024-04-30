function x1 = plant_dt(x0,u0,Ts)

xd0 = plant_ct(x0, u0);
x1 = x0 + xd0 * Ts;

end