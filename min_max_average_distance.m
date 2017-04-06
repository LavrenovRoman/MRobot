function [min, max, average] = min_max_average_distance(pth)

global radii;
global x_center;
global y_center;

min = 1000000000;
max = -100000000;
average = 0;

for i=2:length(pth(:,1))
    minD = 1000000000;
    for j=1:length(radii)
        x = [x_center(j),y_center(j)];
        dist = dist_point_segment(x, pth(i-1,:), pth(i,:));
        dist = dist - radii(j);
        if (dist < minD) 
            minD=dist;
        end;
    end;
    if (minD < min)
        min = minD;
    end;
    if (minD > max)
        max = minD;
    end;
    average = average + minD;
end;

average = average/length(pth(:,1));

end