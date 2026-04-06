function s = make_safe_filename(s)
bad = ['\', '/', ':', '*', '?', '"', '<', '>', '|'];
for i=1:numel(bad), s = strrep(s, bad(i), '_'); end
s = strrep(s,' ','_');
end