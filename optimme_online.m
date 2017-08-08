function optimme_online(n)
%n=-1 - load and draw the result from file
%n=0  - use obstacle map
%n!=0 - interactive obstacle building
WINDOWS = 1;
LINUX = 0;
DEBUG = 0;

close all;
warning off;
format long;

global direct;

global alpha;
global beta;
global number_of_iterations;

global radii;
global x_center;
global y_center;
global circles_intersection;
global front_points;

global intersect_radii;
global intersect_x_center;
global intersect_y_center;

global map_number;
%make place for input points and radii
radii = [];
x=[];
y=[];

front_points = [];

%direct of integral
direct = 2;

%%%%%%%%%%%%%%%%%%%%%%% DATA FOR ITERATIONS %%%%%%%%%%%%%%%%%%%%%%%%
%alpha_vector                = [0.01 0.01 0.01 0.01 0.01      0.01 0.01 0.01 0.01 0.01     0.01 0.01 0.01 0.01 0.01     0.1 0.1 0.1];
alpha_vector                = [0.5 0.6 0.6 0.6 0.6      0.6 0.6 0.6 0.6 0.6     0.7 0.7 0.7 0.7 0.8     0.8 0.8 0.8];
beta_vector                 = [4   4   8   12  16       24  32  64  128 256     32  64  128 256 32      64  128 256];
number_of_iterations_vector = [250 250 250 250 250      200 200 200 200 200     150 150 150 150 150     100 100 100];
filename_vector             = ['a';'b';'c';'d';'e';     'f';'g';'h';'i';'j';    'k';'l';'m';'n';'o';   'p';'q';'r';];

%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%
figure_to_draw = 1;
figure_to_draw_result = 2;
vector_counter = 1;
%%%%%%%%%%%%%%%%%%%% Online - add treatment of intersect later if works! %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if n>0
    %%%%%%%%%%%%%%%%%%%% draw figures %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(WINDOWS)
        draw_obstacle_map(figure_to_draw);
    end
    x_center = zeros(n,1);
    y_center = zeros(n,1);
    radii = zeros(n,1);
    for i=1:n
        [tx,ty] = ginput(2);
        x = [x, tx];
        y = [y, ty];
        t = linspace(0,2*pi,360);
        x_center(i) = x(2*i - 1);
        y_center(i) = y(2*i - 1);
        radii(i) = sqrt((x(2*i - 1) - x(2*i)).^2 + (y(2*i-1) - y(2*i)).^2);
        xx =  x(2*i-1) + cos(t) * radii(i);
        yy =  y(2*i-1) + sin(t) * radii(i);
        fill(xx,yy,'r');
        hold on;
    end
else
    if(n<0)
        load_and_display_result();
    else
        use_obstacle_maps(figure_to_draw); % choose obstacles
    end
end

%%%%%%%%%%%%%%%%%%% S-T %%%%%%%%%%%%%%%%%%%%
if(WINDOWS)
    fprintf('\n Enter Start and Target points with a mouse click\n');
    [stx,sty] = ginput(2);

    stx = stx';
    sty = sty';
    
    stx = [0 100];
    sty = [0 100];

    figure(figure_to_draw);
    plot(stx,sty,'o');
end

limits = [stx(1,1) stx(1,2) sty(1,1) sty(1,2)];
for i=1:length(radii)
    x_c = x_center(1,i);
    y_c = y_center(1,i);
    r_c = radii(1,i);
    if limits(1,1) > x_c - r_c 
        limits(1,1) = x_c - r_c;
    end;
    if limits(1,2) < x_c + r_c 
        limits(1,2) = x_c + r_c;
    end;
    if limits(1,3) > y_c - r_c 
        limits(1,3) = y_c - r_c;
    end;
    if limits(1,4) < y_c + r_c 
        limits(1,4) = y_c + r_c;
    end;
end;
dx = (limits(1,2) - limits(1,1))/20;
dy = (limits(1,4) - limits(1,3))/20;
limits(1,1) = limits(1,1)-dx;
limits(1,2) = limits(1,2)+dx;
limits(1,3) = limits(1,3)-dy;
limits(1,4) = limits(1,4)+dy;

%create points
points_cnt = 3000;
rx=rand(points_cnt,1);
ry=rand(points_cnt,1);
for i=1:points_cnt
    rx(i,1) = (rx(i,1) * (limits(1,2) - limits(1,1))) + limits(1,1);
    ry(i,1) = (ry(i,1) * (limits(1,4) - limits(1,3))) + limits(1,3);
    %plot(rx(i,1),ry(i,1),'o');
end;
for i=points_cnt:-1:1
    for j=1:length(radii)
        x_c = x_center(1,j);
        y_c = y_center(1,j);
        r_c = radii(1,j);
        dist = sqrt((rx(i,1)-x_c)^2 + (ry(i,1)-y_c)^2);
        if dist - r_c < 0.5
            plot(rx(i,1),ry(i,1),'o');
            rx(i) = [];
            ry(i) = [];
            break;
        end;
    end;
end;
points_cnt = length(rx);


start_point = [stx(1),sty(1)];
target_point = [stx(2),sty(2)];

via_points = [(stx(1)+stx(2))./2,(sty(1)+sty(2))./2];

if n>0
    i=1;
    while i<length(radii)
        x_i = x_center(i);
        y_i = y_center(i);
        r_i = radii(i);
        j = 1;
        while j<=length(radii)
            if i==j
                j = j+1;
                continue;
            end
            x_j = x_center(j);
            y_j = y_center(j);
            r_j = radii(j);
            rx = x_i - x_j;
            ry = y_i - y_j;
            dist = sqrt(rx^2 + ry^2);
            if (dist+r_j)<=r_i
                x_center(j) = [];
                y_center(j) = [];
                radii(j) = [];
                break;
            else
                j = j+1;
            end
        end
        if j>length(radii)
            i=i+1;
        end;
    end        
end


tic; 

UseVoronoi = 1;
shortest_path = [];
if UseVoronoi==1 
    intersectionArray = cell(length(radii), 1);
    circles_intersection_points = cell(length(radii), length(radii));
    circles_intersection = cell(length(radii), 1);
    %temp = zeros(2,1);
    %vec = zeros(2,1);
    for i=1:length(radii)
        circles_intersection{i,1} = [circles_intersection{i,1} i];
        x_i = x_center(i);
        y_i = y_center(i);
        r_i = radii(i);
        for j=i+1:length(radii)
            x_j = x_center(j);
            y_j = y_center(j);
            r_j = radii(j);
            rx = x_i - x_j;
            ry = y_i - y_j;
            dist = sqrt(rx^2 + ry^2);
            if dist<r_i+r_j
                %{
                d_i = dist*r_i/(r_i + r_j);
                l = sqrt(r_i^2 - d_i^2);
                temp(1,1) = x_j - x_i;
                temp(2,1) = y_j - y_i;
                x_c = (x_j + x_i)/2;
                y_c = (y_j + y_i)/2;
                if temp(2,1) == 0
                    vec(1,1) = 0;
                    vec(2,1) = 1;
                else 
                    vec(1,1) = 1;
                    vec(2,1) = -1*temp(1,1)/temp(2,1);
                    sum = sqrt(vec(1,1)^2 + vec(2,1)^2);
                    vec(1,1) = vec(1,1)/sum;
                    vec(2,1) = vec(2,1)/sum;
                end;
                xs = zeros(2,1);
                ys = zeros(2,1);
                xs(1,1) = x_c + vec(1,1)*l;
                xs(2,1) = x_c - vec(1,1)*l;
                ys(1,1) = y_c + vec(2,1)*l;
                ys(2,1) = y_c - vec(2,1)*l;
                %}
                syms x y;
                h1=(y - y_center(i))^2 + (x - x_center(i))^2 - radii(i)^2;
                h2=(y - y_center(j))^2 + (x - x_center(j))^2 - radii(j)^2;
                [xx, yy]=solve(h1,h2,'Real',true);
                circles_intersection_points{i,j} = [double(xx(1,1)) double(yy(1,1))];
                circles_intersection_points{j,i} = [double(xx(2,1)) double(yy(2,1))];
                intersectionArray{i,1} = [intersectionArray{i,1} j];
                intersectionArray{j,1} = [i intersectionArray{j,1}];
                circles_intersection{i,1} = [circles_intersection{i,1} j];
                circles_intersection{j,1} = [i circles_intersection{j,1}];
            end;
        end;
    end;

    it = 1;
    while it < length(circles_intersection)
        findI = 0;
        for i=it+1:length(circles_intersection)
            Inter = intersect(circles_intersection{it,1}, circles_intersection{i,1});
            if ~isempty(Inter)
                circles_intersection{it,1} = union(circles_intersection{it,1}, circles_intersection{i,1});
                circles_intersection(i) = [];
                findI = i;
                break;
            end;
        end;
        if findI == 0
            it = it + 1;
        end;
    end; 

    front_points = cell(length(radii)^2, 1);
    f_p = 1;
    step = 1;
    b = zeros(2,1);
    for i=1:length(radii)
        t = ceil(2*pi*radii(i)/step); 
        d = ceil(360/t);
        endCircPoint = 0;
        temp = zeros(2,1);
        beginBad = -1;
        for a=d:d:(360+d)
            x = x_center(i) + radii(i)*cos(degtorad(a));
            y = y_center(i) + radii(i)*sin(degtorad(a));
            findC = 0;
            for c=1:length(intersectionArray{i,1})
                circ = intersectionArray{i,1}(1,c);
                dist = sqrt((x-x_center(circ))^2 + (y-y_center(circ))^2);
                if (dist < radii(circ))
                    findC = circ;
                    break;
                end;
            end;
            b(1,1) = x;
            b(2,1) = y;
            if findC == 0
                if temp(1,1)~=0 && temp(2,1)~=0
                    front_points{f_p,1} = [front_points{f_p,1} temp];
                    temp = zeros(2,1);
                end;
                front_points{f_p,1} = [front_points{f_p,1} b];
                endCircPoint = 1;            
                if a == d
                    beginBad = f_p;
                end;
            else
                dist1 = sqrt((x-circles_intersection_points{i, findC}(1,1))^2 + (y-circles_intersection_points{i, findC}(1,2))^2);
                dist2 = sqrt((x-circles_intersection_points{findC, i}(1,1))^2 + (y-circles_intersection_points{findC, i}(1,2))^2);
                if dist1<dist2
                    temp(1,1) = circles_intersection_points{i, findC}(1,1);
                    temp(2,1) = circles_intersection_points{i, findC}(1,2);
                else
                    temp(1,1) = circles_intersection_points{findC,i}(1,1);
                    temp(2,1) = circles_intersection_points{findC,i}(1,2);
                end;
                if endCircPoint == 1
                    front_points{f_p,1} = [front_points{f_p,1} temp];
                    f_p = f_p + 1;
                    temp = zeros(2,1);
                    endCircPoint = 0;
                end;                
            end;
        end;
        if beginBad ~= -1
            if beginBad~=f_p 
                front_points{beginBad,1} = [front_points{f_p,1} front_points{beginBad,1}];
                front_points(f_p) = [];
            else
                f_p = f_p + 1;
                %front_points{beginBad,1} = [front_points{beginBad,1}(:, length(front_points{beginBad,1})) front_points{beginBad,1}];
            end;
        end;
    end;

    for i=length(front_points):-1:1
        if isempty(front_points{i, 1})
            front_points(i) = [];
        end;
    end;

    f_p_it = 1;
    while f_p_it < length(front_points)
        findI = 0;
        i = f_p_it+1;
        while i<length(front_points)+1
            if isempty(front_points{i, 1})
                break;
            end;
            point = front_points{f_p_it, 1}(:,length(front_points{f_p_it, 1}));
            p1 = front_points{i, 1}(:,1);
            if p1 == point
                front_points{i, 1}(:,1) = [];
                front_points{f_p_it, 1} = [front_points{f_p_it, 1} front_points{i, 1}];
                front_points(i) = [];
                findI = 1;
                break;
            end;
            point = front_points{f_p_it, 1}(:,1);
            p2 = front_points{i, 1}(:,length(front_points{i, 1}));
            if p2 == point
                front_points{f_p_it, 1}(:,1) = [];
                front_points{f_p_it, 1} = [front_points{i, 1} front_points{f_p_it, 1} ];
                front_points(i) = [];
                findI = 1;
                break;
            end;
            i = i + 1;
        end;
        if findI == 0
            f_p_it = f_p_it + 1;
        end;
    end;

    clf(figure_to_draw);
    draw_obstacle_map(figure_to_draw);

    while length(front_points) > length(circles_intersection)
        minFP = 10000000;
        minFPi = 0;
        for i=1:length(front_points)
            if minFP > length(front_points{i, 1})
                minFP = length(front_points{i, 1});
                minFPi = i;
            end;
        end;
        front_points(minFPi) = [];
    end;

    clf(figure_to_draw);
    draw_obstacle_map(figure_to_draw);

%    limits = [stx(1,1) stx(1,2) sty(1,1) sty(1,2)];
%    for i=1:length(front_points)
%        for j=1:length(front_points{i, 1})
%            point = front_points{i, 1}(:,j);
%            if limits(1,1) > point(1,1) 
%                limits(1,1) = point(1,1);
%            end;
%            if limits(1,2) < point(1,1) 
%                limits(1,2) = point(1,1);
%            end;
%            if limits(1,3) > point(2,1) 
%                limits(1,3) = point(2,1);
%            end;
%            if limits(1,4) < point(2,1) 
%                limits(1,4) = point(2,1);
%            end;
%        end;
%    end;
%    dx = (limits(1,2) - limits(1,1))/20;
%    dy = (limits(1,4) - limits(1,3))/20;
%    limits(1,1) = limits(1,1)-dx;
%    limits(1,2) = limits(1,2)+dx;
%    limits(1,3) = limits(1,3)-dy;
%    limits(1,4) = limits(1,4)+dy;
    
    
    s_x = linspace(limits(1,1),limits(1,2),101);
    s_y = linspace(limits(1,3),limits(1,4),101);
    
    fp = zeros(length(s_x),length(s_y));
    
    show_potent_field=0;
    if (show_potent_field==1)
        sh_beta = 4;
        sh_alpha = 0.5;
        for i=1:length(s_x)
            for j=1:length(s_y)
                for k=1:length(radii)

                x_0 = x_center(k);
                y_0 = y_center(k);
                r_0 = radii(k);

                rx = s_x(i) - x_0;
                ry = s_y(j) - y_0;
                dist = r_0 - sqrt(rx^2 + ry^2);

                fp(i,j) = fp(i,j) + sh_beta *(1+tanh(sh_alpha * dist));
                %f2 = max(f2,beta *(1+tanh(alpha * dist)));
                end;
            end;
        end;
        figure
        mesh(fp)
    end;    

    epsilonvoronoi = step;

    obstacles = front_points';
    for i=1:length(obstacles)
        obstacles{1, i} = obstacles{1, i}';
    end;

    [X_Total_points,Y_Total_points, All_cells_Number, Cell_start, X1] = rmt_voronoi_epsi(length(circles_intersection), limits,epsilonvoronoi,obstacles);

    [trajDV, Vertex_Cord_DV, PathWithoutCurve, CostWithoutCurve, ...
                        VertWithoutCurve, Edges, Verts] = rmt_get_voronoi(limits, (length(circles_intersection)+1), start_point, ...
                        target_point, X_Total_points, Y_Total_points, ...
                        All_cells_Number, Cell_start, X1, 1);

    k = 1;
    shortest_path(1, 1) = trajDV(1,1);
    shortest_path(2, 1) = trajDV(1,2);
    for i=2:length(trajDV(:,1))
        p1 = [shortest_path(k, 1) shortest_path(k+1, 1)];
        p2 = [trajDV(i, 1) trajDV(i, 2)];
        for c=1:length(radii)
            x = [x_center(c) y_center(c)];
            r_c = radii(c);
            dist = dist_point_segment(x, p1, p2);
            if dist<r_c
                k = k + 2;
                shortest_path(k, 1) = trajDV(i-1,1);
                shortest_path(k+1, 1) = trajDV(i-1,2);
                break;
            end;
        end;
    end;
    shortest_path(1, :) = [];
    shortest_path(1, :) = [];
end

t1=toc;
t1

tic; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIRST ITERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alpha = alpha_vector(vector_counter);
beta = beta_vector(vector_counter);
%filename = filename_vector(vector_counter);
%number_of_iterations = 200;
number_of_iterations = number_of_iterations_vector(vector_counter);

if isempty(shortest_path)
    [new_points, spline_xyt]  = add_one_more_point_to_spline(via_points, start_point, target_point, figure_to_draw);
else
    [new_points, spline_xyt]  = add_one_more_point_to_spline(shortest_path, start_point, target_point, figure_to_draw);
end;
new_points
if(WINDOWS)
    print('-dbmp16m', filename_vector(vector_counter));
end
new_solution_cost = evaluate_solution(spline_xyt);
%old_solution_cost = new_solution_cost;
previous_iteration_spline_xyt = spline_xyt;

if(DEBUG)
    input('Press ENTER to continue to the next part');
end
vector_counter = vector_counter+1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CICLE OF ITERATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for vector_counter = 2:length(alpha_vector)
    alpha = alpha_vector(vector_counter);
    beta = beta_vector(vector_counter);
    %filename =filename_vector(vector_counter);
    %number_of_iterations = 20;
    number_of_iterations = number_of_iterations_vector(vector_counter);

    via_points = new_points;
    [new_points,spline_xyt] = add_one_more_point_to_spline(via_points, start_point, target_point, figure_to_draw);
    new_points
    old_solution_cost = new_solution_cost;
    if(DEBUG)
        input('\nPress ENTER to continue to the next part\n');
    end
    new_solution_cost  = evaluate_solution(spline_xyt);

    if( (check_solution(previous_iteration_spline_xyt,start_point,target_point,new_solution_cost, ...
            old_solution_cost,figure_to_draw, figure_to_draw_result))  > 0)
        if(WINDOWS)
            print('-dbmp16m', 'result');
        end
        if(LINUX)
            iteration_number = vector_counter-1;
            save('result', 'previous_iteration_spline_xyt','start_point','target_point','n','map_number','iteration_number');
        end
        
        t2=toc;
        t2
        
        return;
    end

    if(DEBUG)
        input('\nPress ENTER to continue to the next part\n');
    end
   
    if(WINDOWS)
        print('-dbmp16m', filename_vector(vector_counter));
    end
    close(figure_to_draw);
    previous_iteration_spline_xyt = spline_xyt;

end

t2=toc;
t2

next = input('\nFirst stage search failed. To continue with second search stage press "1". To stop - press "0"\n');

if(next)
    beta_vector                 = [4     8    16   32   8      16  32  64  16   32     64  128 32  64  128     192 256 64  128 192      256 384 512 32  64      128 192 256 384 512   64  128 192 256 384   512];
    alpha_vector                = [0.05  0.05 0.05 0.05 0.1    0.1 0.1 0.1 0.3  0.3    0.3 0.3 0.5 0.5 0.5     0.5 0.5 0.6 0.6 0.6      0.6 0.6 0.6 0.7 0.7     0.7 0.7 0.7 0.7 0.7   0.8 0.8 0.8 0.8 0.8   0.8];
    number_of_iterations_vector = [250   250  250  250  250    200 200 200 200  200    200 200 200 200 200     150 150 150 150 150      150 150 150 150 150     100 100 100 100 100   100 100 100 100 100   100];
    filename_vector             = ['t';  'u'; 'v'; 'w'; 'x';   'y';'z';'a';'b';'c';    'd';'e';'f';'g';'h';    'i';'j';'k';'l';'m';     'n';'o';'p';'q';'r';    't';'u';'v';'w';'x';  'y';'z';'a';'b';'c';  'd';];

    via_points = [(stx(1)+stx(2))./2,(sty(1)+sty(2))./2];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIRST ITERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    previous_vector_counter = vector_counter;
    vector_counter = 1;
    alpha = alpha_vector(vector_counter);
    beta = beta_vector(vector_counter);
    %filename = filename_vector(vector_counter);
    number_of_iterations = number_of_iterations_vector(vector_counter);

    [new_points, spline_xyt]  = add_one_more_point_to_spline(via_points, start_point, target_point, figure_to_draw);
    if(WINDOWS)
        print('-dbmp16m', filename_vector(vector_counter));
    end
    new_solution_cost = evaluate_solution(spline_xyt);
    %old_solution_cost = new_solution_cost;
    previous_iteration_spline_xyt = spline_xyt;

    if(DEBUG)
        input('Press ENTER to continue to the next part');
    end
    %vector_counter = vector_counter+1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% CICLE OF ITERATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for vector_counter = 2:length(alpha_vector)
        alpha = alpha_vector(vector_counter);
        beta = beta_vector(vector_counter);
        %filename =filename_vector(vector_counter);
        number_of_iterations = number_of_iterations_vector(vector_counter);

        via_points = new_points;
        [new_points,spline_xyt] = add_one_more_point_to_spline(via_points, start_point, target_point, figure_to_draw);
        old_solution_cost = new_solution_cost;
        new_solution_cost  = evaluate_solution(spline_xyt);

        if( (check_solution(previous_iteration_spline_xyt,start_point,target_point,new_solution_cost, ...
                old_solution_cost,figure_to_draw, figure_to_draw_result))  > 0)
            if(WINDOWS)
                print('-dbmp16m', 'result');
            end
            if(LINUX)
                iteration_number = vector_counter - 1 + previous_vector_counter;
                save('result', 'previous_iteration_spline_xyt','start_point','target_point','n','map_number','iteration_number');
            end
            return;
        end

        if(DEBUG)
            input('\nPress ENTER to continue to the next part\n');
        end
        if(WINDOWS)
            print('-dbmp16m', filename_vector(vector_counter));
        end
        close(figure_to_draw);
        previous_iteration_spline_xyt = spline_xyt;
    end
end

input('\nSecond stage search failed. Press ENTER to exit :(\n');

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%% DREK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%