function t_k_parts = arc_K_parts(k,spline_xyt)
%%%%%%%%%%%%% next victim for optimization - 4 points 17 sec, 5 - 29, 6 -
%%%%%%%%%%%%% 30, 7 - 34 vs 2549
L = arc_length_integral(0,2,spline_xyt);
k_points = [0];
a = k;
b = 1;
while(a>1)   
    result = fminsearch(@K_middle,(2-k_points(b))/a,[],L,k_points(b),a,spline_xyt);    
    k_points = [k_points result];% t-coordinates  
    b = b+1;
    a = a-1;
    L = arc_length_integral(k_points(b),2,spline_xyt);    
%arc_length_integral(k_points(b-1),k_points(b),spline_xyt)
end

t_k_parts = k_points(2:end);