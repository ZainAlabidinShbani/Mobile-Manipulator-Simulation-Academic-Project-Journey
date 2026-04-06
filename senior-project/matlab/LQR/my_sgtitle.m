function my_sgtitle(txt)
if exist('sgtitle','file') == 2
    sgtitle(txt);
else
    annotation('textbox',[0 0.95 1 0.05], ...
        'String',txt,'EdgeColor','none', ...
        'HorizontalAlignment','center', ...
        'FontWeight','bold');
end
end
