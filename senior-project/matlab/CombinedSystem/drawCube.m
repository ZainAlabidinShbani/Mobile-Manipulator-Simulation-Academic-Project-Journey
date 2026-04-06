function h = drawCube(ax, center, s)
c = center(:); d = s/2;
V = [ -d -d -d;
       d -d -d;
       d  d -d;
      -d  d -d;
      -d -d  d;
       d -d  d;
       d  d  d;
      -d  d  d ] + c';
F = [1 2 3 4;
     5 6 7 8;
     1 2 6 5;
     2 3 7 6;
     3 4 8 7;
     4 1 5 8];
h.patch = patch(ax,'Vertices',V,'Faces',F,'FaceAlpha',0.7,'EdgeAlpha',0.2);
end