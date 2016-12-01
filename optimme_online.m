function optimme_online(n)
%n=-1 - load and draw the result from file
%n=0  - use obstacle map
%n!=0 - interactive obstacle building
WINDOWS = 1;
LINUX = 0;
DEBUG = 1;

close all;
warning off;
format long;

global alpha;
global beta;
global number_of_iterations;

global radii;
global x_center;
global y_center;

global intersect_radii;
global intersect_x_center;
global intersect_y_center;

global map_number;
%make place for input points and radii
radii = [];
x=[];
y=[];

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
