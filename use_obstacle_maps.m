function use_obstacle_maps(figure_to_draw);

global radii;
global x_center;
global y_center;

global intersect_radii;
global intersect_x_center;
global intersect_y_center;

global map_number;

intersect_x_center = [];
intersect_y_center = [];
intersect_radii = [];

%%%%%%%%%%%%%%%%%%%%% Offline %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf('\n\t*************************************************************************');
fprintf('\n\t*\t\tEnter the figure number                                           *');
fprintf('\n\t*\t1 - 1 huge circle                                                   *');
fprintf('\n\t*\t2 - 2 circles                                                       *');
fprintf('\n\t*\t3 - full park park                                                  *');
fprintf('\n\t*\t4 - upper left corner                                               *');
fprintf('\n\t*\t5 - small path                                                      *');
fprintf('\n\t*\t61 - 2 big circles - average size intersection                                             *');
fprintf('\n\t*\t62 - 2 big circles - very small size intersection                                             *');
fprintf('\n\t*\t63 - 2 intersecting circles in the center of the map                                          *');
fprintf('\n\t*\t64 - 2 intersecting circles in the center of the map                                          *');
fprintf('\n\t*\t7 - 3 circles intersect                                             *');
fprintf('\n\t*\t8 - romb                                                            *');
fprintf('\n\t*\t9 - mountain pass                                                   *');
fprintf('\n\t*\t10 - mountain pass with local minimum                               *');
fprintf('\n\t*\t11 - groups with local minimum                                      *');
fprintf('\n\t*\t12 - small central park                                             *');
fprintf('\n\t*\t13 - small central park +                                           *');
fprintf('\n\t*\telse - closed obstacle                                              *');
fprintf('\n\t*************************************************************************');
map_number = input('\n>');

obstacle_maps();

draw_obstacle_map(figure_to_draw);
return;