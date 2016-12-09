function [new_points, spline_xyt] = add_one_more_point_to_spline(via_points, start, target, figure_to_draw )
DEBUG_TIC_TOC = 0;
middle_point_plot = 0;

figure(figure_to_draw);
close(figure_to_draw);

global number_of_iterations;

%hold on;
draw_obstacle_map(figure_to_draw);
hold on;

if(DEBUG_TIC_TOC)
    tic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

if (number_of_iterations > 0) %for initial splines
    [result,fval,exitflag ]= fminsearch(@potent_K, ...
        via_points,... %initial guess
        optimset('MaxIter',number_of_iterations,'Display', 'on','TolX',0.1),...%    [], 'off'
        start(1),start(2),target(1),target(2));
else % for precise splines
    [result,fval,exitflag ]= fminsearch(@potent_K, ...
        via_points,... %initial guess
        [],start(1),start(2),target(1),target(2));
end
%%%%%%%%%%%%%%%%%% final spline %%%%%%%%%%%%%%%%%
if(DEBUG_TIC_TOC)
    toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
%tic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

via_points = result;
final_x = [start(1)];
final_y = [start(2)];
number_of_via_points = length(via_points)/2;

for i=1:number_of_via_points
    final_x = [final_x via_points(2*i-1)];
    final_y = [final_y via_points(2*i)];
end
final_x = [final_x target(1)];
final_y = [final_y target(2)];
t = linspace(0,2,number_of_via_points+2);

spline_xyt = spline(t, [final_x; final_y]);

plot_final_spline(spline_xyt,start,target,figure_to_draw,0);

%toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%tic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%% looking for the K-middle points %%%%%%%%%%%%
%L = arc_length_integral(0,2, spline_xyt); %Modify Roman

%toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if(DEBUG_TIC_TOC)
    tic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

K_middle_points = arc_K_parts(number_of_via_points+2,spline_xyt);

if(DEBUG_TIC_TOC)
    toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
%tic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

number_of_K_middle_points = length(K_middle_points);

%toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%tic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

middle_x =[];
middle_y =[];
temp_points = zeros(number_of_K_middle_points*2,1);
for i=1:number_of_K_middle_points
    spline_xyt_value = ppval(spline_xyt,K_middle_points(i));
    t_middle_x = spline_xyt_value(1);
    t_middle_y = spline_xyt_value(2);    
    temp_points(2*i-1) = t_middle_x;
    temp_points(2*i) = t_middle_y;
    if(middle_point_plot)
        middle_x =[middle_x t_middle_x];
        middle_y =[middle_y t_middle_y];
    end;
end

if(middle_point_plot)
    plot(middle_x,middle_y,'o');
end
%toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

new_points = temp_points;
