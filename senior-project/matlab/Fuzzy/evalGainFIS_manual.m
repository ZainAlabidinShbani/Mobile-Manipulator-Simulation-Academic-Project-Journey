function alpha = evalGainFIS_manual(en, den, mode)
% Manual Mamdani-like fuzzy inference WITHOUT Fuzzy Logic Toolbox
% Inputs: en, den in [-1,1]
% Output: alpha in [0,1]

en  = max(-1, min(1, en));
den = max(-1, min(1, den));

% 7 input MFs: NB NM NS Z PS PM PB
centers = linspace(-1,1,7);
step = centers(2)-centers(1);
w = step;  % half-width for triangles

mu_e  = zeros(1,7);
mu_de = zeros(1,7);
for k=1:7
    c = centers(k);
    a = c - w; b = c; d = c + w;
    mu_e(k)  = trimf_local(en,  a,b,d);
    mu_de(k) = trimf_local(den, a,b,d);
end

% Output alpha MFs: VS S M L VL
outCenters = [0.10 0.30 0.50 0.70 0.90];
outW = 0.20;

u_grid = linspace(0,1,501);
mu_agg = zeros(size(u_grid));

% Rule evaluation + aggregation
for ie = 1:7
    for ide = 1:7
        w_rule = min(mu_e(ie), mu_de(ide));
        if w_rule <= 0, continue; end

        outIdx = ruleConsequentIndex(ie, ide, mode); % 1..5
        c = outCenters(outIdx);
        a = max(0, c-outW);
        b = c;
        d = min(1, c+outW);

        mu_out = arrayfun(@(u) trimf_local(u,a,b,d), u_grid);
        mu_agg = max(mu_agg, min(w_rule, mu_out));
    end
end

% Defuzzification centroid
num = trapz(u_grid, u_grid .* mu_agg);
denom = trapz(u_grid, mu_agg);

if denom < 1e-12
    alpha = 0.5;
else
    alpha = num/denom;
end
alpha = max(0, min(1, alpha));
end