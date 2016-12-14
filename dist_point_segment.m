function dist = dist_point_segment(x, a, b)
%DIST_POINT_SEGMENT Summary of this function goes here
%   Detailed explanation goes here

%x = [0,0]; %some point
%a = [1,2]; %segment points a,b
%b = [3,5];

d_ab = norm(a-b);
d_ax = norm(a-x);
d_bx = norm(b-x);

if dot(a-b,x-b)*dot(b-a,x-a)>=0
    A = [a,1;b,1;x,1];
    dist = abs(det(A))/d_ab;        
else
    dist = min(d_ax, d_bx);
end

end

