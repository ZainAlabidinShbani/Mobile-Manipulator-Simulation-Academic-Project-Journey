function icon = make_icon(kind)
icon = 0.92*ones(16,16,3);
icon(:,:,1) = icon(:,:,1)*0.95;
icon(:,:,2) = icon(:,:,2)*0.95;
icon(:,:,3) = icon(:,:,3)*0.98;

switch lower(kind)
    case '+'
        icon(8:9,4:13,:) = 0.15;
        icon(4:13,8:9,:) = 0.15;
    case '<'
        for k=1:6
            icon(8-k, 6+k, :) = 0.15;
            icon(8+k, 6+k, :) = 0.15;
        end
        icon(8:9,6:13,:) = 0.15;
    case 'ok'
        for k=0:4
            icon(10+k, 4+k,:) = 0.15;
        end
        for k=0:6
            icon(14-k, 9+k,:) = 0.15;
        end
end
end