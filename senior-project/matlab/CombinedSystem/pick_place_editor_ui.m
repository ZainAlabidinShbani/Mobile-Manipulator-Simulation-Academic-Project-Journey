function [pick_xy, place_xy] = pick_place_editor_ui(world, obstacles, pick_init, place_init)
pick_xy  = pick_init;
place_xy = place_init;

fig = figure('Name','Pick & Place Editor (Click to set)', 'NumberTitle','off');
ax = axes(fig); hold(ax,'on'); grid(ax,'on');
axis(ax,'equal'); xlim(ax,world.xlim); ylim(ax,world.ylim);
xlabel(ax,'x'); ylabel(ax,'y');
title(ax,'Use buttons: Set PICK / Set PLACE / Done');

plot(ax, [world.xlim(1) world.xlim(2) world.xlim(2) world.xlim(1) world.xlim(1)], ...
         [world.ylim(1) world.ylim(1) world.ylim(2) world.ylim(2) world.ylim(1)], '-', 'LineWidth',1);

for j=1:size(obstacles,1)
    draw_circle(ax, obstacles(j,1), obstacles(j,2), obstacles(j,3));
end

hPick  = plot(ax, pick_xy(1),  pick_xy(2),  'go', 'MarkerSize',9, 'LineWidth',2);
hPlace = plot(ax, place_xy(1), place_xy(2), 'ro', 'MarkerSize',9, 'LineWidth',2);

legend(ax, {'world','obstacles','PICK','PLACE'}, 'Location','bestoutside');

btnW=110; btnH=34; pad=8;

uicontrol(fig,'Style','pushbutton','String','Set PICK', ...
    'Position',[pad pad btnW btnH], 'Callback',@onSetPick);

uicontrol(fig,'Style','pushbutton','String','Set PLACE', ...
    'Position',[pad+btnW+pad pad btnW btnH], 'Callback',@onSetPlace);

uicontrol(fig,'Style','pushbutton','String','Swap', ...
    'Position',[pad+2*(btnW+pad) pad btnW btnH], 'Callback',@onSwap);

uicontrol(fig,'Style','pushbutton','String','Reset', ...
    'Position',[pad+3*(btnW+pad) pad btnW btnH], 'Callback',@onReset);

uicontrol(fig,'Style','pushbutton','String','Done', ...
    'Position',[pad+4*(btnW+pad) pad btnW btnH], 'Callback',@onDone);

txt = uicontrol(fig,'Style','text','String',statusLine(), ...
    'Position',[pad pad+btnH+6 560 22], 'HorizontalAlignment','left');

uiwait(fig);

    function onSetPick(~,~)
        title(ax,'Click PICK point (green)...');
        [x,y,btn] = ginput(1);
        if isempty(btn), title(ax,'Pick & Place Editor'); return; end
        if ~inWorld(x,y), title(ax,'Outside world. Try again.'); return; end
        pick_xy = [x;y];
        set(hPick,'XData',x,'YData',y);
        set(txt,'String',statusLine());
        title(ax,'Pick set. You can set Place or Done.');
    end

    function onSetPlace(~,~)
        title(ax,'Click PLACE point (red)...');
        [x,y,btn] = ginput(1);
        if isempty(btn), title(ax,'Pick & Place Editor'); return; end
        if ~inWorld(x,y), title(ax,'Outside world. Try again.'); return; end
        place_xy = [x;y];
        set(hPlace,'XData',x,'YData',y);
        set(txt,'String',statusLine());
        title(ax,'Place set. Done when ready.');
    end

    function onSwap(~,~)
        tmp = pick_xy; pick_xy = place_xy; place_xy = tmp;
        set(hPick,'XData',pick_xy(1),'YData',pick_xy(2));
        set(hPlace,'XData',place_xy(1),'YData',place_xy(2));
        set(txt,'String',statusLine());
        title(ax,'Swapped.');
    end

    function onReset(~,~)
        pick_xy  = pick_init;
        place_xy = place_init;
        set(hPick,'XData',pick_xy(1),'YData',pick_xy(2));
        set(hPlace,'XData',place_xy(1),'YData',place_xy(2));
        set(txt,'String',statusLine());
        title(ax,'Reset to defaults.');
    end

    function onDone(~,~)
        if isvalid(fig)
            uiresume(fig);
            close(fig);
        end
    end

    function ok = inWorld(x,y)
        ok = (x>=world.xlim(1) && x<=world.xlim(2) && y>=world.ylim(1) && y<=world.ylim(2));
    end

    function s = statusLine()
        s = sprintf('PICK = [%.2f, %.2f]   |   PLACE = [%.2f, %.2f]', ...
            pick_xy(1), pick_xy(2), place_xy(1), place_xy(2));
    end
end
