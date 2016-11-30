function load_and_display_result()

global radii;
global x_center;
global y_center;

global intersect_radii;
global intersect_x_center;
global intersect_y_center;

global map_number;
% make place for input points and radii
%radii = [];
%x=[];
%y=[];

figure_to_draw_result = 1;

iteration_number = 0;

% save('result','previous_iteration_spline_xyt','start_point','target_point','n','map_number');
load('C:\RFFI\MRobot\result.mat');
obstacle_maps();
draw_obstacle_map(figure_to_draw_result);
hold on;
plot_final_spline(previous_iteration_spline_xyt,start_point,target_point,figure_to_draw_result,1);
fprintf('\n The solution is found in %d steps. \n', iteration_number);
next = input('Press ENTER to exit\n');