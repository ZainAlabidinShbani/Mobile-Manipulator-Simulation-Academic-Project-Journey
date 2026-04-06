function setCubePose(h, center, s)
c = center(:); d = s/2;
V = [ -d -d -d;
       d -d -d;
       d  d -d;
      -d  d -d;
      -d  d  d;
       d -d  d;
       d  d  d;
      -d  d  d ] + c';
set(h.patch,'Vertices',V);
end