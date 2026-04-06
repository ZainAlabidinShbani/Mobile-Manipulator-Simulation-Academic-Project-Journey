function y = clamp_vec(x, xmin, xmax)
% elementwise clamp: xmin/xmax can be scalars or vectors
y = min(max(x, xmin), xmax);
end