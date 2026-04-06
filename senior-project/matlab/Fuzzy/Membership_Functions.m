%% ====== FUZZY MEMBERSHIP FUNCTIONS PLOTS (FOR REPORT) ======
centers = linspace(-1,1,7);
w = centers(2)-centers(1);
x = linspace(-1,1,1201);

labels_in = {'NB','NM','NS','Z','PS','PM','PB'};

% Input MFs (same for e_n and de_n)
mu = zeros(7,numel(x));
for k=1:7
    c = centers(k);
    a = c-w; b=c; d=c+w;
    for ii=1:numel(x)
        mu(k,ii) = trimf_local(x(ii),a,b,d);
    end
end

figure('Color','w'); hold on; grid on;
for k=1:7
    plot(x,mu(k,:),'LineWidth',1.3);
end
xlabel('x \in [-1,1]'); ylabel('\mu(x)');
title('Input Membership Functions (for e_n or de_n)');
legend(labels_in,'Location','best');

% Output MFs
outCenters = [0.10 0.30 0.50 0.70 0.90];
outW = 0.20;
u = linspace(0,1,1201);
labels_out = {'VS','S','M','L','VL'};

muo = zeros(5,numel(u));
for k=1:5
    c = outCenters(k);
    a = max(0,c-outW); b=c; d=min(1,c+outW);
    for ii=1:numel(u)
        muo(k,ii) = trimf_local(u(ii),a,b,d);
    end
end

figure('Color','w'); hold on; grid on;
for k=1:5
    plot(u,muo(k,:),'LineWidth',1.3);
end
xlabel('\alpha \in [0,1]'); ylabel('\mu(\alpha)');
title('Output Membership Functions for Gain Scalers (\alpha)');
legend(labels_out,'Location','best');
%% ====== FUZZY MEMBERSHIP FUNCTIONS PLOTS (FOR REPORT) ======
centers = linspace(-1,1,7);
w = centers(2)-centers(1);
x = linspace(-1,1,1201);

labels_in = {'NB','NM','NS','Z','PS','PM','PB'};

% Input MFs (same for e_n and de_n)
mu = zeros(7,numel(x));
for k=1:7
    c = centers(k);
    a = c-w; b=c; d=c+w;
    for ii=1:numel(x)
        mu(k,ii) = trimf_local(x(ii),a,b,d);
    end
end

figure('Color','w'); hold on; grid on;
for k=1:7
    plot(x,mu(k,:),'LineWidth',1.3);
end
xlabel('x \in [-1,1]'); ylabel('\mu(x)');
title('Input Membership Functions (for e_n or de_n)');
legend(labels_in,'Location','best');

% Output MFs
outCenters = [0.10 0.30 0.50 0.70 0.90];
outW = 0.20;
u = linspace(0,1,1201);
labels_out = {'VS','S','M','L','VL'};

muo = zeros(5,numel(u));
for k=1:5
    c = outCenters(k);
    a = max(0,c-outW); b=c; d=min(1,c+outW);
    for ii=1:numel(u)
        muo(k,ii) = trimf_local(u(ii),a,b,d);
    end
end

figure('Color','w'); hold on; grid on;
for k=1:5
    plot(u,muo(k,:),'LineWidth',1.3);
end
xlabel('\alpha \in [0,1]'); ylabel('\mu(\alpha)');
title('Output Membership Functions for Gain Scalers (\alpha)');
legend(labels_out,'Location','best');
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
