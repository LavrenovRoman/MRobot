function f = halakut_K(t,spline_xyt)

ddspline_xyt = fnder(spline_xyt,2); %second derivative of the spline

ddspline_xyt_value = ppval(ddspline_xyt,t);
x = ddspline_xyt_value(1,:);
y = ddspline_xyt_value(2,:);

f = x.^2 +  y.^2;