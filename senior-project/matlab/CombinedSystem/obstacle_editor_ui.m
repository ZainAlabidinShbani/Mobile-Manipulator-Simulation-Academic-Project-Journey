function obstacles = obstacle_editor_ui(world, obstacles_init)
obstacles = obstacles_init;

fig = figure('Name','Obstacle Editor (Click to place)', 'NumberTitle','off');
ax = axes(fig); hold(ax,'on'); grid(ax,'on');
axis(ax,'equal'); xlim(ax,world.xlim); ylim(ax,world.ylim);
xlabel(ax,'x'); ylabel(ax,'y');
title(ax,'Add obstacles: click inside map. Use buttons: Add / Undo / Done');

plot(ax, [world.xlim(1) world.xlim(2) world.xlim(2) world.xlim(1) world.xlim(1)], ...
         [world.ylim(1) world.ylim(1) world.ylim(2) world.ylim(2) world.ylim(1)], '-', 'LineWidth',1);

hObs = gobjects(0);

iconAdd  = make_icon('+');
iconUndo = make_icon('<');
iconDone = make_icon('ok');

btnW=90; btnH=34; pad=8;

uicontrol(fig,'Style','pushbutton','String',' Add','CData',iconAdd, ...
    'Position',[pad pad btnW btnH], 'Callback',@onAdd);

uicontrol(fig,'Style','pushbutton','String',' Undo','CData',iconUndo, ...
    'Position',[pad+btnW+pad pad btnW btnH], 'Callback',@onUndo);

uicontrol(fig,'Style','pushbutton','String',' Done','CData',iconDone, ...
    'Position',[pad+2*(btnW+pad) pad btnW btnH], 'Callback',@onDone);

txt = uicontrol(fig,'Style','text','String','Radius default: 0.25m', ...
    'Position',[pad+3*(btnW+pad) pad 180 btnH], 'HorizontalAlignment','left');

defaultR = 0.25;
redraw();
uiwait(fig);

    function onAdd(~,~)
        prompt = {'Obstacle radius (m):'};
        dlgtitle = 'Add obstacle';
        dims = [1 35];
        def = {num2str(defaultR)};
        answ = inputdlg(prompt, dlgtitle, dims, def);
        if isempty(answ), return; end
        r = str2double(answ{1});
        if ~isfinite(r) || r<=0, r = defaultR; end
        defaultR = r;
        set(txt,'String',sprintf('Radius default: %.2fm', defaultR));

        title(ax,'Click location for obstacle center...');
        [x,y,btn] = ginput(1);
        if isempty(btn), redraw(); return; end
        if x<world.xlim(1) || x>world.xlim(2) || y<world.ylim(1) || y>world.ylim(2)
            redraw(); return;
        end
        obstacles = [obstacles; x y r];
        redraw();
    end

    function onUndo(~,~)
        if ~isempty(obstacles)
            obstacles(end,:) = [];
            redraw();
        end
    end

    function onDone(~,~)
        if isvalid(fig)
            uiresume(fig);
            close(fig);
        end
    end

    function redraw()
        if ~isempty(hObs)
            for i=1:numel(hObs)
                if isvalid(hObs(i)), delete(hObs(i)); end
            end
        end
        hObs = gobjects(0);
        for j=1:size(obstacles,1)
            hObs(end+1) = draw_circle(ax, obstacles(j,1), obstacles(j,2), obstacles(j,3)); %#ok<AGROW>
        end
        title(ax,'Add obstacles: click inside map. Use buttons: Add / Undo / Done');
        drawnow;
    end
end