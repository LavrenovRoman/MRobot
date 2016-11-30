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

t = linspace(0,2,via_points+2); %all together via_points+2 points

spline_xyt = spline(t, [points_x; points_y]);
spline_xyt_ti = linspace(0,2,400);
spline_xyt_xyt = ppval(spline_xyt,spline_xyt_ti);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

obst =  quad('potent_cost_K',0,2,[],[],spline_xyt);
hal = sqrt(quad('halakut_K',0,2,[],[],spline_xyt));
len = 0.5*quad('arc_length',0,2,[],[],spline_xyt);
%len = 0;
%hal = 0;
%0.01*
cost1 = obst + hal + len;

fprintf('obstacles = %9g    length = %9g  halakut = %9g ALL = %9g\n', obst,  len, hal, cost1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% plot to see the process :
if(DEBUG)
    array_x = spline_xyt_xyt(1,:);
    array_y = spline_xyt_xyt(2,:);
    plot(array_x,array_y, 'b-', 'LineWidth',2);
    %next = input('\nPress ENTER to continue to the next part\n');
end

cost = cost1;
