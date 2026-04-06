function mu = trimf_local(x,a,b,c)
% Triangular membership function (no toolbox)
if x <= a || x >= c
    mu = 0;
elseif x == b
    mu = 1;
elseif x < b
    mu = (x-a)/(b-a + eps);
else
    mu = (c-x)/(c-b + eps);
end
mu = max(0,min(1,mu));
end