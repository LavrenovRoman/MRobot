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

global intersect_radii;
global intersect_x_center;
global intersect_y_center;

global map_number;
%make place for input points and radii
radii = [];
x=[];
y=[];

%direct of integral
direct = 2;

%%%%%%%%%%%%%%%%%%%%%%% DATA FOR ITERATIONS %%%%%%%%%%%%%%%%%%%%%%%%
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
    %start_point = [stx(1),sty(1)]
    %target_point = [stx(2),sty(2)]
    %plot((stx(1)+stx(2))./2,(sty(1)+sty(2))./2,'o');
end
%stx(1)= -3;  sty(1)=29; stx(2)= 80; sty(2)=51; %for park
%stx(1)= -9;  sty(1)=44; stx(2)= 110; sty(2)=42; %for fig.9
% stx(1)=-17; sty(1)=52; stx(2)=127; sty(2)=62;  %for fig.8
%stx(1)=-2;  sty(1)=42; stx(2)= 98; sty(2)=67;  %fig 10
%stx(1)=-2;  sty(1)=44; stx(2)= 98; sty(2)=67;

% stx(1)=22; sty(1)=28; stx(2)=71; sty(2)=77; %fig 63
%stx(1)= 52; sty(1)=36; stx(2)=71; sty(2)=73; %fig 63
start_point = [stx(1),sty(1)];
target_point = [stx(2),sty(2)];

via_points = [(stx(1)+stx(2))./2,(sty(1)+sty(2))./2];

intersectionArray = cell(length(radii), 1);
intersectionMatrix = zeros(length(radii), length(radii));
circles_intersection_points = cell(length(radii), length(radii));
circles_intersection = cell(length(radii), 1);
if ~isempty(radii)
    circles_intersection{1,1} = [circles_intersection{1,1} 1];
end;
for i=1:length(radii)
    x_i = x_center(i);
    y_i = y_center(i);
    r_i = radii(i);
    findSome = 0;
    for j=i+1:length(radii)
        x_j = x_center(j);
        y_j = y_center(j);
        r_j = radii(j);
        rx = x_i - x_j;
        ry = y_i - y_j;
        dist = sqrt(rx^2 + ry^2);
        if dist<=r_i+r_j
            syms x y;
            h1=(y - y_center(i))^2 + (x - x_center(i))^2 - radii(i)^2;
            h2=(y - y_center(j))^2 + (x - x_center(j))^2 - radii(j)^2;
            [xx yy]=solve(h1,h2);
            circles_intersection_points{i,j} = [double(xx(1,1)) double(yy(1,1))];
            circles_intersection_points{j,i} = [double(xx(2,1)) double(yy(2,1))];
            intersectionArray{i,1} = [intersectionArray{i,1} j];
            intersectionArray{j,1} = [intersectionArray{j,1} i];            
            intersectionMatrix(i,j) = 1;
            intersectionMatrix(j,i) = 1;
            findI = 0;
            for k=1:length(circles_intersection)
                for l=1:length(circles_intersection{k,1})
                    if circles_intersection{k,1}(1,l) == i
                        findI = 1;
                        circles_intersection{k,1} = [circles_intersection{k,1} j];
                        break;
                    end;
                end;
                if findI == 1 
                    break;
                end;
            end;
            if findI == 0
                circles_intersection{i,1} = [circles_intersection{i,1} i];
                circles_intersection{i,1} = [circles_intersection{i,1} j];
            end;
            findSome = 1;
        end;
    end;
    if findSome == 0
        findI = 0;
        for k=1:length(circles_intersection)
            for l=1:length(circles_intersection{k,1})
                if circles_intersection{k,1}(1,l) == i
                    findI = 1;
                    break;
                end;
            end;
            if findI == 1 
                break;
            end;
        end;
        if findI == 0
            circles_intersection{i,1} = [circles_intersection{i,1} i];
        end;
    end;
end;

for i=length(circles_intersection):-1:1
    if isempty(circles_intersection{i,1})
        circles_intersection(i) = [];
        continue;
    else
        circles_intersection{i,1} = unique(circles_intersection{i,1});
    end;
end;

circ = 1;
while circ < length(circles_intersection)
    findC = 0;
    for i=circ+1:length(circles_intersection)
        Inter = intersect(circles_intersection{circ,1}, circles_intersection{i,1});
        if ~isempty(Inter)
            findC = i;
            break;
        end;
    end;
    if findC == 0
        circ = circ + 1;
    else
        circles_intersection{circ,1} = union(circles_intersection{circ,1}, circles_intersection{i,1});
        circles_intersection(i) = [];
    end;
end;

front_points_circles = cell(length(radii), 1);
step = 1;
b = zeros(2,1);
temp = zeros(2,1);
for i=1:length(radii)
    t = ceil(2*pi*radii(i)/step); 
    d = ceil(360/t);
    for a=d:d:360
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
                front_points_circles{i,1} = [front_points_circles{i,1} temp];
                temp = zeros(2,1);
            end;
            front_points_circles{i,1} = [front_points_circles{i,1} b];
        else
            dist1 = sqrt((x-circles_intersection_points{i, findC}(1,1))^2 + (y-circles_intersection_points{i, findC}(1,2))^2);
            dist2 = sqrt((x-circles_intersection_points{findC, i}(1,1))^2 + (y-circles_intersection_points{findC, i}(1,2))^2);
            if dist1<dist2
                temp(1,1) = circles_intersection_points{i, findC}(1,1);
                temp(2,1) = circles_intersection_points{i, findC}(1,2);
            else
                temp(1,1) = circles_intersection_points{findC,1}(1,1);
                temp(2,1) = circles_intersection_points{findC,1}(1,2);
            end;
        end;
    end;
end;

%graph_of_intersections = cell(length(circles_intersection), 1);
%{
for i=1:length(circles_intersection)
    if length(circles_intersection{i,1}) == 1
    else
        for j=1:length(circles_intersection{i,1})-1
            indC1 = circles_intersection{i,1}(1,j);
            for k=j+1:length(circles_intersection{i,1})
                indC2 = circles_intersection{i,1}(1,k);
                if intersectionMatrix(indC1,indC2) == 1
                    
                    
                end;
            end;   
        end;            
    end;
end;
%}

draw_obstacle_map(figure_to_draw);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIRST ITERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alpha = alpha_vector(vector_counter);
beta = beta_vector(vector_counter);
%filename = filename_vector(vector_counter);
%number_of_iterations = 200;
number_of_iterations = number_of_iterations_vector(vector_counter);

[new_points, spline_xyt]  = add_one_more_point_to_spline(via_points, start_point, target_point, figure_to_draw);
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
