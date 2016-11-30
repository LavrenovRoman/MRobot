function start_from_step_k(n,k_step)
%n=figure number
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
number_of_iterations_vector = [200 200 200 200 200      200 150 150 150 150     100 100 100 100 100     100 100 100];
filename_vector             = ['a';'b';'c';'d';'e';     'f';'g';'h';'i';'j';    'k';'l';'m';'n';'o';   'p';'q';'r';];

%%%%%%%%%%%%%%%%%% INITIALIZATION %%%%%%%%%%%%%
figure_to_draw = 1;
figure_to_draw_result = 2;
vector_counter = 1;
%%%%%%%%%%%%%%%%%%%% Online - add treatment of intersect later if works! %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
map_number = n;
obstacle_maps();
draw_obstacle_map(figure_to_draw); % choose obstacles
vector_counter =  k_step; 
%%%%%%%%%%%%%%%%%%% S-T %%%%%%%%%%%%%%%%%%%%

%stx(1)= -3;  sty(1)=29; stx(2)= 80; sty(2)=51; %for park
%stx(1)= -9;  sty(1)=44; stx(2)= 110; sty(2)=42; %for fig.9
%stx(1)=-2;  sty(1)=44; stx(2)= 98; sty(2)=67;
%figure 8

if(n==8)

stx(1)=-17; sty(1)=52; stx(2)=127; sty(2)=62;  %for fig.8

%step 2
via_points = [38.24982575453771,  59.26746583149766,   93.60115205738357,   65.70228213427386]; %format: [x1,y1,x2,y2,x3,y3]
%step 3
via_points = 1.0e+02 *[ 0.23679780965680,    0.57587743851146,    0.64434346809920,   0.62591908892009,   1.05306208616916,   0.66526647161636];
%step 4
via_points = 1.0e+02 * [0.09492964368503,   0.65098009932402,   0.41077365751384,   0.63066539638173,   0.72636515098414,   0.60539047794995,   1.03233323277701,   0.67357946478479];
%step 5
via_points = 1.0e+02 * [ 0.02469982630031,   0.67549564730078,   0.30618554446998,   0.60573740744081,   0.59575933347865,   0.59572777756670,   0.88395017581805,   0.62894514720400,   1.15936078306432,   0.71685496693249];
%step 6
via_points =  1.0e+02 * [ 0.00922760406319,   0.67852918592754,   0.23599886314756,   0.61044551773676,   0.46916663956760,   0.55421233641404,   0.70828759128892,   0.55198456782893,   0.93162257938979,   0.63696805720203,   1.13873906978117,   0.75801067098713];
%step 7
via_points = 1.0e+02 * [ -0.07020416573849,   0.68620418343893,   0.13084630214993,   0.65858031452243,   0.27303292396287,   0.53061344334559,   0.48045109756075,   0.53580227521395,   0.68688981359031,   0.55756084063695,   0.88749519829142,   0.60838984765087,   1.07302433743983,   0.68314571041363];
%step 8
via_points = 1.0e+02 * [ -0.06073427082013,   0.65695478974321,   0.11893312282732,   0.66988091279120,   0.24770130582503,   0.55990237608725,   0.40045728398734,   0.61822364119179,   0.58540306539607,   0.60859718719366,   0.77011886645900,   0.59509148909354,   0.93988431565987,   0.64342825075330,   1.10999754680442,   0.70474715112742];
%step 9
via_points = 1.0e+02 * [ -0.06751770728124,   0.65250281405672,   0.09431236632975,   0.69312248457901,   0.22621366401850,   0.58626179778761,   0.36699741042337,   0.66958929145355,   0.49577072059150,   0.61274537468745,   0.65754760807361,   0.55637917747744,   0.82798858166106,   0.55105383205247,   0.95176427742624,   0.66424517565132,   1.11084473181018,   0.68487343978516];
%step 10
via_points = 1.0e+02 * [ -0.08659347746123,   0.65371198124933,   0.04989980022880,   0.72329851255997,   0.17185282830752,   0.62859596796718,   0.29270784078512,   0.60186912614673,   0.42629894911802,   0.65590454546917,   0.54521452352644,   0.58820845457185,   0.70301601956829,   0.57259830550960,   0.86008254190527,   0.58616607218903,   0.98030209676696,   0.68595734046058,   1.12643913041435,   0.68741990021376];
%step 11
via_points = 1.0e+02 * [ -0.07602201405498,   0.63441656956461,   0.04392982598640,   0.71418951061022,   0.16666026536036,   0.63740750021224,   0.28891250372117,   0.59654496761195,   0.39580681997651,   0.69916662104102,   0.48673800694435,   0.62416925638127,   0.63333627942562,   0.60378883714491,   0.78156517298819,   0.59813786965533,   0.87718208407907,   0.58881321766243,   0.99585156670555,   0.67674512245022,   1.13861553316759,   0.68724928933195];
%step 12
via_points = 1.0e+02 * [ -0.09804673139142,   0.62875622170487,   0.01331850925995,   0.69098462507599,   0.13457077525537,   0.64772079483288,   0.25298085269741,   0.59718292441973,   0.36673264690149,   0.65858583877933,   0.47104797042750,   0.62084325134047,   0.58767618972796,   0.61462357680923,   0.70986191131681,   0.65818775014048,   0.81966628210162,   0.59435523465589,   0.92402974535063,   0.62491012451539,   1.02551433689108,   0.70188074408259,   1.15203049340236,   0.67638044914895];

end
start_point = [stx(1),sty(1)];
target_point = [stx(2),sty(2)];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIRST ITERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alpha = alpha_vector(vector_counter);
beta = beta_vector(vector_counter);
filename = filename_vector(vector_counter);
%number_of_iterations = 200;
number_of_iterations = number_of_iterations_vector(vector_counter);

[new_points, spline_xyt]  = add_one_more_point_to_spline(via_points, start_point, target_point, figure_to_draw);
if(WINDOWS)
    print('-dbmp16m', filename_vector(vector_counter));
end
new_solution_cost = evaluate_solution(spline_xyt);
old_solution_cost = new_solution_cost;
previous_iteration_spline_xyt = spline_xyt;

if(DEBUG)
    next = input('Press ENTER to continue to the next part');
end
vector_counter = vector_counter+1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CICLE OF ITERATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for vector_counter = (k_step+1):length(alpha_vector)
    alpha = alpha_vector(vector_counter);
    beta = beta_vector(vector_counter);
    filename =filename_vector(vector_counter);
    %number_of_iterations = 20;
    number_of_iterations = number_of_iterations_vector(vector_counter);

    via_points = new_points;
    [new_points,spline_xyt] = add_one_more_point_to_spline(via_points, start_point, target_point, figure_to_draw);
    old_solution_cost = new_solution_cost;
    if(DEBUG)
        next = input('\nPress ENTER to continue to the next part\n');
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
        next = input('\nPress ENTER to continue to the next part\n');
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
    number_of_iterations_vector = [200   200  200  200  100    100 100 100 100  100    100 100 100 100 100     50  50  50  50  50       50  50  50  50  50      50  50  50  50  50    50  50  50  50  50    50 ];
    filename_vector             = ['t';  'u'; 'v'; 'w'; 'x';   'y';'z';'a';'b';'c';    'd';'e';'f';'g';'h';    'i';'j';'k';'l';'m';     'n';'o';'p';'q';'r';    't';'u';'v';'w';'x';  'y';'z';'a';'b';'c';  'd';];

    via_points = [(stx(1)+stx(2))./2,(sty(1)+sty(2))./2];
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FIRST ITERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    previous_vector_counter = vector_counter;
    vector_counter = 1;
    alpha = alpha_vector(vector_counter);
    beta = beta_vector(vector_counter);
    filename = filename_vector(vector_counter);
    number_of_iterations = number_of_iterations_vector(vector_counter);

    [new_points, spline_xyt]  = add_one_more_point_to_spline(via_points, start_point, target_point, figure_to_draw);
    if(WINDOWS)
        print('-dbmp16m', filename_vector(vector_counter));
    end
    new_solution_cost = evaluate_solution(spline_xyt);
    old_solution_cost = new_solution_cost;
    previous_iteration_spline_xyt = spline_xyt;

    if(DEBUG)
        next = input('Press ENTER to continue to the next part');
    end
    vector_counter = vector_counter+1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% CICLE OF ITERATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    for vector_counter = 2:length(alpha_vector)
        alpha = alpha_vector(vector_counter);
        beta = beta_vector(vector_counter);
        filename =filename_vector(vector_counter);
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
            next = input('\nPress ENTER to continue to the next part\n');
        end
        if(WINDOWS)
            print('-dbmp16m', filename_vector(vector_counter));
        end
        close(figure_to_draw);
        previous_iteration_spline_xyt = spline_xyt;
    end
end

next = input('\nSecond stage search failed. Press ENTER to exit :(\n');

return;

%%%%%%%%%%%%%%%%%%%%%%%%%%%% DREK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
