function L = arc_length_integral(end1,end2, spline_xyt)
%end1 = zeros(length(end2),1)'
%end2
	L = quad(@arc_length,end1, end2, [], [], spline_xyt);