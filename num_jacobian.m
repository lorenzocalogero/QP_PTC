function J = num_jacobian(f, x)

epsilon = 1e-06;
n = length(x);
m = length(f(x));

J = zeros(m, n);

for i = 1:1:n
	dx = zeros(n,1);
	dx(i) = epsilon;
	J(:,i) = (f(x + dx) - f(x - dx)) ./ (2 * epsilon);
end

end