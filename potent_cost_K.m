function f= potent_cost_K(t,spline_xyt)

DEBUG = 0;
KICK_KNAS = 0; %if to pay for crossing the obstacle
take_into_account_intersection_as_1_point = 0;

global alpha;
global beta;

global radii;
global x_center;
global y_center;

global intersect_radii;
global intersect_x_center;
global intersect_y_center;

obstacles_crossed = zeros(length(radii), 1);

spline_xyt_value = ppval(spline_xyt,t);
x = spline_xyt_value(1,:);
y = spline_xyt_value(2,:);

dspline_xyt = fnder(spline_xyt);
dspline_xyt_value = ppval(dspline_xyt,t);
%dx = spline_xyt_value(1,:);
%dy = spline_xyt_value(2,:);
dx = dspline_xyt_value(1,:); %Modify Roman
dy = dspline_xyt_value(2,:); %Modify Roman

f1 = sqrt((dx).^2 + (dy).^2);
f2 = f1 * 0;

for j=1:length(radii)

    x_0 = x_center(j);
    y_0 = y_center(j);
    r_0 = radii(j);
    
    rx = zeros(1,length(x));
    ry = zeros(1,length(x));
    dist = zeros(1,length(x));

    for counter=1:length(x)
        rx(counter) = x(counter) - x_0;
        ry(counter) = y(counter) - y_0;
        dist(counter) = r_0 - sqrt(rx(counter).^2 + ry(counter).^2);

        if (KICK_KNAS && (dist(counter) < 0))
            obstacles_crossed(j) = 1;
        end
    end;

    f2 = f2 + beta *(1+tanh(alpha * dist));
    %f2 = max(f2,beta *(1+tanh(alpha * dist)));
end

if(KICK_KNAS)
    f_obstacles_crossed = 0;
    f_obstacles_crossed = sum (obstacles_crossed);
    f_obstacles_crossed = f_obstacles_crossed * beta * 100;
    if(DEBUG)
        fprintf('weight = %9g\n', f_obstacles_crossed);
    end;
end;

f3 = f1 * 0;
if(take_into_account_intersection_as_1_point)
    for j=1:length(intersect_radii)-1

        x_0 = intersect_x_center(j);
        y_0 = intersect_y_center(j);
        r_0 = intersect_radii(j);

        j = j+1;

        x_1 = intersect_x_center(j);
        y_1 = intersect_y_center(j);
        r_1 = intersect_radii(j);

        x_i = (x_0 + x_1)/2;
        y_i = (y_0 + y_1)/2;
        r_i = max (r_0 + sqrt((x_0-x_i).^2 + (y_0-y_i).^2), ...
            r_1 + sqrt((x_1-x_i).^2 + (y_1-y_i).^2)) ;


        for counter=1:length(x)
            if((x(counter)>0 & x_i>0) | (x(counter)<0 & x_i<0))
                rx(counter) = x(counter) - x_i;
            else
                rx(counter) = x(counter) + x_i;
            end;

            if((x(counter)>0 & x_0>0) | (x(counter)<0 & x_0<0))
                rx0(counter) = x(counter) - x_0;
            else
                rx0(counter) = x(counter) + x_0;
            end;

            if((x(counter)>0 & x_1>0) | (x(counter)<0 & x_1<0))
                rx1(counter) = x(counter) - x_1;
            else
                rx1(counter) = x(counter) + x_1;
            end;


            if((y(counter)>0 & y_i>0) | (y(counter)<0 & y_i<0))
                ry(counter) = y(counter) - y_i;
            else
                ry(counter) = y(counter) + y_i;
            end;

            if((y(counter)>0 & y_0>0) | (y(counter)<0 & y_0<0))
                ry0(counter) = y(counter) - y_0;
            else
                ry0(counter) = y(counter) + y_0;
            end;

            if((y(counter)>0 & y_1>0) | (y(counter)<0 & y_1<0))
                ry1(counter) = y(counter) - y_1;
            else
                ry1(counter) = y(counter) + y_1;
            end;

            dist(counter) =  r_i - sqrt(rx(counter).^2 + ry(counter).^2);
            dist0(counter) =  r_0 - sqrt(rx0(counter).^2 + ry0(counter).^2);
            dist1(counter) =  r_1 - sqrt(rx1(counter).^2 + ry1(counter).^2);

            if(dist1(counter)>=0 || dist0(counter)>=0)
                f3(counter) = f3(counter) + beta *(1+tanh(alpha * dist(counter)));
            else
                f3(counter) = f3(counter) + 1*beta *(1+tanh(alpha * dist(counter)));
                %next = input('\nPress ENTER to continue to the next part\n');
            end


            if (KICK_KNAS && (dist(counter) < 0))
                obstacles_crossed(j) = 1;
                obstacles_crossed(j-1) = 1;
            end

        end;


        %f3 = f3 + beta *(1+tanh(alpha * dist));
        %f2 = max(f2,beta *(1+tanh(alpha * dist)));
    end
end
if(KICK_KNAS)
    f_obstacles_crossed = 0;
    f_obstacles_crossed = sum (obstacles_crossed);
    f_obstacles_crossed = f_obstacles_crossed * beta * 100;
    if(DEBUG)
        fprintf('weight = %9g\n', f_obstacles_crossed);
    end;
end;

%next = input('\nPress ENTER to continue to the next part\n');
if(KICK_KNAS)
    f = f1.*(f2+f3) + f_obstacles_crossed;
else
    f = f1.*(f2+f3);
end

