function L = arc_length(t,spline_xyt)

dspline_xyt = fnder(spline_xyt);
dspline_xyt_value = ppval(dspline_xyt,t);
dx = dspline_xyt_value(1,:);
dy = dspline_xyt_value(2,:);

L = sqrt(dx.^2+dy.^2);