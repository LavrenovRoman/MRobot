function f = vis_start_cost(path)

global radii;
global x_center;
global y_center;

f = 0;
for i=2:length(path(:,1))
    intersectObst = 0;
    for j=1:length(radii)
        x = [x_center(j),y_center(j)];
        dist = dist_point_segment(x, path(1,:), path(i,:));
        if (dist < radii(j))
            intersectObst = 1;
            break;
        end;
    end;
    if intersectObst == 0
        f = f + sqrt((path(i,1)-path(i-1,1))^2 + (path(i,2)-path(i-1,2))^2);
    end;
end;
end