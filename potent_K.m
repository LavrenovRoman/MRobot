function cost=potent_K(x,xS,yS,xT,yT)
DEBUG = 0;
%tic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%% build t-spline for descendants too %%%%%%%%%%%%
via_points = length(x)/2;

i = 1:via_points;
points_x = zeros(via_points+2,1);
points_y = zeros(via_points+2,1);
points_x(i+1) = x(i*2-1);
points_y(i+1) = x(i*2);
points_x(1) = xS;
points_y(1) = yS;
points_x(via_points+2) = xT;
points_y(via_points+2) = yT;
points_x = points_x';
points_y = points_y';

global direct;

t = linspace(0,direct,via_points+2); %all together via_points+2 points

spline_xyt = spline(t, [points_x; points_y]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

distcount = 20;
step = (spline_xyt.breaks(1,2) - spline_xyt.breaks(1,1))/distcount;
path = zeros(1+distcount*(length(spline_xyt.breaks)-1), 2);

lengthpath = 0;
p = 1;
path(p,1) = spline_xyt.coefs(1, spline_xyt.order);
path(p,2) = spline_xyt.coefs(2, spline_xyt.order);
p = p + 1;

for i=1:length(spline_xyt.breaks)-1
    for j=1:distcount
        a = j*step;
        x = 0;
        y = 0;
        for k=spline_xyt.order:-1:1
            x = x + spline_xyt.coefs((i*2)-1, k)*(a^(spline_xyt.order-k));
            y = y + spline_xyt.coefs((i*2)  , k)*(a^(spline_xyt.order-k));
        end
        path(p,1) = x;
        path(p,2) = y;        
        lengthpath = lengthpath + sqrt((x-path(p-1,1))^2 + (y-path(p-1,2))^2);
        p = p + 1;
    end
end

%toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%hoefV = 1;

%[vmin, vmax, vaverage] = min_max_average_distance(path);

%safe_dist = 3;
%gamma = 20;
%c_min = gamma^(safe_dist-vmin);

%vstart = hoefV*(100 - (100*vis_start_cost(path) /lengthpath));
%vtargt = hoefV*(100 - (100*vis_target_cost(path)/lengthpath));
obst =  quad('potent_cost_K',0,direct,[],[],spline_xyt);
hal = sqrt(quad('halakut_K',0,direct,[],[],spline_xyt));
len = 0.5*quad('arc_length',0,direct,[],[],spline_xyt);
%len = 0;
%hal = 0;
%0.01*
cost = obst + hal + len; % + 0*vstart + 0*vtargt + 0*c_min;

%for i=1:length(x(:,1))
%    fprintf('%9g ', x(i,1));
%end;
%fprintf('\n');

fprintf('obstacles = %9g  length = %9g  halakut = %9g  ALL = %9g\n', obst, len, hal, cost);
%fprintf('obstacles = %9g  length = %9g  halakut = %9g  vstart = %9g  vtargt = %9g  ALL = %9g\n', obst, len, hal, vstart, vtargt, cost);
%fprintf('obstacles = %9g  length = %9g  halakut = %9g  min = %9g  c_min = %9g  ALL = %9g\n', obst, len, hal, vmin, c_min, cost);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot to see the process :
if(DEBUG)
    spline_xyt_ti = linspace(0,2,400);
    spline_xyt_xyt = ppval(spline_xyt,spline_xyt_ti);
    array_x = spline_xyt_xyt(1,:);
    array_y = spline_xyt_xyt(2,:);
    plot(array_x,array_y, 'b-', 'LineWidth',2);
    %next = input('\nPress ENTER to continue to the next part\n');
end
