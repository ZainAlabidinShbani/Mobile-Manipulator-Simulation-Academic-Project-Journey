function log = trim_logs(log,kend)
if kend <= 0
    fields = fieldnames(log);
    for i=1:numel(fields), log.(fields{i}) = []; end
    return;
end
fn = fieldnames(log);
for i=1:numel(fn)
    v = log.(fn{i});
    if isvector(v)
        log.(fn{i}) = v(1:kend);
    else
        log.(fn{i}) = v(:,1:kend);
    end
end
end