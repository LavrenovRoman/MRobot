function plot_final_spline(spline_xyt,start,target,figure_to_draw,final)

%figure(figure_to_draw);
%close(figure_to_draw);
%hold on;
figure(figure_to_draw);
if(final)
    draw_obstacle_map(figure_to_draw);
end
hold on;

spline_xyt_ti = linspace(0,2);
spline_xyt_xyt = ppval(spline_xyt,spline_xyt_ti);

plot([start(1),target(1)],[start(2),target(2)],'o');
plot(spline_xyt_xyt(1,:),spline_xyt_xyt(2,:), 'g-', 'LineWidth',4);

return;