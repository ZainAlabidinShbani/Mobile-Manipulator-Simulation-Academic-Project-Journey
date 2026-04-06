
function V = cube_vertices(center,s)
cx=center(1); cy=center(2); cz=center(3);
d = s/2;
V = [cx-d cy-d cz-d;
     cx+d cy-d cz-d;
     cx+d cy+d cz-d;
     cx-d cy+d cz-d;
     cx-d cy-d cz+d;
     cx+d cy-d cz+d;
     cx+d cy+d cz+d;
     cx-d cy+d cz+d];
end