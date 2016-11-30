function mid = K_middle(t,L,from,k,spline_xyt)

middle = t;
%mid = max(abs(L/k - arc_length_integral(from,t,spline_xyt)), ...
  % abs((k-1)*L/k - arc_length_integral(t,2,spline_xyt)));
   mid = abs(L/k - arc_length_integral(from,t,spline_xyt));