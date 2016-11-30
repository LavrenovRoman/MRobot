function cost = evaluate_solution(spline_xyt)
DEBUG = 1;
global alpha;
global beta;

global radii;
global x_center;
global y_center;

global intersect_radii;
global intersect_x_center;
global intersect_y_center;
%%%%%%%%%%%%%%%%% checking intersections %%%%%%%%%%%%%%%%%

epsilon = -0.5;

cost_length = quad('arc_length',0, 2, [], [], spline_xyt);
discretization = 0.1;
parts = double(int16(cost_length/discretization));
t = linspace(0,2,parts-1);

spline_xyt_value = ppval(spline_xyt,t);
x = spline_xyt_value(1,:);
y = spline_xyt_value(2,:);

%start = [x(1) y(1)];
%target = [x(length(x)) y(length(y))];
%plot_final_spline(spline_xyt,start,target,3);

for j=1:length(radii)

    x_0 = x_center(j);
    y_0 = y_center(j);
    r_0 = radii(j);

    for counter=1:length(x)
        rx(counter) = x(counter) - x_0;
            ry(counter) = y(counter) - y_0;
        
        dist(counter) =  sqrt(rx(counter).^2 + ry(counter).^2) - r_0;

        if (dist(counter) < epsilon)
            if(DEBUG)
                fprintf('\nintersection at x = %9g y = %9g \n Obstacle x0 = %9g y0 = %9g \n Distance to the boundary dist = %9g \n', ...
                    x(counter), y(counter),x_center(j),y_center(j),dist(counter));                
            end
            cost = -1;
            return;
        end
    end;
end

for j=1:length(intersect_radii)

    x_0 = intersect_x_center(j);
    y_0 = intersect_y_center(j);
    r_0 = intersect_radii(j);

    for counter=1:length(x)
            rx(counter) = x(counter) - x_0;
            ry(counter) = y(counter) - y_0;
        
        dist(counter) =  sqrt(rx(counter).^2 + ry(counter).^2) - r_0;

        if (dist(counter) < epsilon)
            if(DEBUG)
                fprintf('\nCross-intersection at x = %9g y = %9g \n Obstacle x0 = %9g y0 = %9g \n Distance to the boundary dist = %9g \n', ...
                    x(counter), y(counter),x_center(j),y_center(j),dist(counter));                
            end
            cost = -1;
            return;
        end
    end;
end

%tic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%store original Alpha and Beta
original_alpha = alpha;
original_beta = beta;
%use this Alpha and Beta for all evaluations
alpha = 2;
beta = 10;

cost_obstacles  = quad('potent_cost_K',0,2,[],[],spline_xyt);
cost_halakut = sqrt(quad('halakut_K',0,2,[],[],spline_xyt));

%restore original Alpha and Beta
alpha = original_alpha;
beta = original_beta;

if(DEBUG)
    fprintf('\nEVALUATION:\nobstacles = %9g    length = %9g  halakut = %9g ALL = %9g\n', cost_obstacles ,  cost_length, cost_halakut, cost_halakut + cost_length);
end
%cost = cost1 + cost2 + cost3;
cost = cost_halakut + cost_length;