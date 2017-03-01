function draw_obstacle_map(figure_to_draw)

%tic %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global radii;
global x_center;
global y_center;

global intersect_radii;
global intersect_x_center;
global intersect_y_center;

global front_points;

%vizual stuff
axis_xmin = -40;
axis_xmax = 160;
axis_ymin = -40;
axis_ymax = 160;

frame_xmin = axis_xmin - 2;
frame_xmax = axis_xmax + 2;
frame_ymin = axis_ymin - 2;
frame_ymax = axis_ymax + 2;
frame_xpoints = 1 * (frame_xmax - frame_xmin); %instead of 1 use 10 later
frame_ypoints = 1 * (frame_ymax - frame_ymin);

figure(figure_to_draw);
fx = linspace(frame_xmin,frame_xmax,frame_xpoints);
fy = linspace(frame_ymin,frame_ymax,frame_ypoints);

%frame for workspace
%plot(fx, frame_ymin, 'k');
%plot(fx, frame_ymax, 'k');
%plot(frame_xmin, fy, 'k');
%plot(frame_xmax, fy, 'k');

%axis choice
axis manual tight equal;
axis([frame_xmin-1 frame_xmax+1 frame_ymin-1 frame_ymax+1]);

hold on;
for i=1:length(front_points)
    x = front_points{i,1}(1,:);
    y = front_points{i,1}(2,:);
    plot(x,y, 'r');
end;

if isempty(front_points)
    for i=1:length(radii)
        t = linspace(0,2*pi,72);%instead of 72 use 360 later
        xx =  x_center(i) + cos(t) * radii(i); 
        yy =  y_center(i) + sin(t) * radii(i);
        fill(xx,yy,'r');
        hold on;  
    end;
end;

for i=1:length(intersect_radii)
      t = linspace(0,2*pi,72);%instead of 72 use 360 later
      xx =  intersect_x_center(i) + cos(t) * intersect_radii(i); 
      yy =  intersect_y_center(i) + sin(t) * intersect_radii(i);
      fill(xx,yy,'r');
      hold on;  
end
   %toc %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   TIC-TOC   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
return;