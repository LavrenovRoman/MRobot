%    This is part of RMTool - Robot Motion Toolbox, for Matlab 2010b or newer.
%
%    Copyright (C) 2016 RMTool developing team. For people, details and citing 
%    information, please see: http://webdiis.unizar.es/RMTool/index.html.
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program.  If not, see <http://www.gnu.org/licenses/>.

%% ============================================================================
%   MOBILE ROBOT TOOLBOX
%   Graphical User Interface
%   First version released on September, 2014. 
%   Last modification December 29, 2015.
%   More information: http://webdiis.unizar.es/RMTool
% ============================================================================


function [traj, Vertex_Cord, PathWithoutCurve, CostWithoutCurve, VertWithoutCurve, Edges, Verts] = rmt_get_voronoi(limits, ...
    Num_Object, Start, Goal, X_Total_points, Y_Total_points, All_cells_Number, Cell_start, X1, Is_draw)

%clear all;
%close all;
%clc;

traj = [];
Nxi = limits(1);
Nx = limits(2);
Nyi = limits(3);
Ny = limits(4);
%env_bounds=[Nxi,Nx,Nyi,Ny];

%axes(handle_axes);
%axis(env_bounds);
%hold on
%grid on


%specify file name here to load from
%LOAD_FILE_NAME = 'Obstacle_config';

%load(strcat(strcat(LOAD_FILE_NAME)));



%Code for drawing obstale configuration
%for i=1:Num_Object
%    for r=1:length(X1{i})
%       a=r;
%       if(r==length(X1{i}))
%           b=1;
%       else
%           b=r+1;
%       end
%       x=[X1{i}(a,1) X1{i}(b,1)];
%       y=[X1{i}(a,2) X1{i}(b,2)];
%       plot(x,y,'Color', 'Black');
%       hold on;
%    end
%end

%Code for taking Start and End point as input
%Start = ginput(1);
%plot(Start(1),Start(2),'--go','MarkerSize',10,'MarkerFaceColor','g');
%drawnow;
%Goal  = ginput(1);
%plot(Goal(1),Goal(2),'--ro','MarkerSize',10,'MarkerFaceColor','r');
%drawnow;

%Uncomment following to Draw voronoi diagram of point obstacles
%voronoi(X_Total_points,Y_Total_points);
%Getting Parameters of Voronoi Diagram
[Voro_Vertex,Voro_Cell] = voronoin([X_Total_points' Y_Total_points']);

%Temp_Edge = zeros(length(All_cells_Number)*2, 2);
k=1;
%temp=0;
for i=1:length(All_cells_Number)
    L=length(Voro_Cell{i});
  for j=1:L
      a=Voro_Cell{i}(j);
      if(j==L)
          b=Voro_Cell{i}(1);
      else
          b=Voro_Cell{i}(j+1);
      end
            
      if (a==b)
          continue;
      end
            
      x1 = Voro_Vertex(a,1);
      y1 = Voro_Vertex(a,2);
      if ((x1>Nx)||(x1<Nxi)||(y1>Ny)||(y1<Nyi)||isinf(x1)||isnan(x1)||isinf(y1)||isnan(y1))
          continue;
      end
      x2 = Voro_Vertex(b,1);
      y2 = Voro_Vertex(b,2);
      if ((x2>Nx)||(x2<Nxi)||(y2>Ny)||(y2<Nyi)||isinf(x2)||isnan(x2)||isinf(y2)||isnan(y2))
          continue;
      end
      for l=1:Num_Object
          %if(temp==1)
          %    temp=0;
          %    break;
          %end
          if ((k>1)&&(Temp_Edge(k-1,1)==a)&&(Temp_Edge(k-1,2)==b))
              break;
          end
          if (l==All_cells_Number(i))
              if (l~=1)
                 continue;
              end
          end
          
          if (l==1)
              mBegin = Cell_start(l);
          else
              mBegin = Cell_start(l)-1;
          end;
                            
          for m=mBegin:Cell_start(l+1)-2%-2
              if(~isempty(find(Voro_Cell{m}==a, 1)))&&(~isempty(find(Voro_Cell{m}==b, 1)))
                  if (l==All_cells_Number(i) && l==1 && abs(m-i)>1)||(l~=All_cells_Number(i))
                      Temp_Edge(k,:)=[a b];
                      k=k+1;
                      %temp=1;
                      break;
                  end
              end
          end     
      end
  end    
end

Temp_Edge=unique(Temp_Edge,'rows');   % Temp_Edge - индексы пар точек из [X_Total_points' Y_Total_points']  
%Temp_Edge(1,:) = [];

%Delete duplicate edges like (1, 2) and (2, 1)
temp1 = 1;
while temp1<length(Temp_Edge)
    temp2 = temp1 + 1;
    while temp2<length(Temp_Edge)+1
        if ((Temp_Edge(temp1,1) == Temp_Edge(temp2,2))&&(Temp_Edge(temp2,1) == Temp_Edge(temp1,2)))
             Temp_Edge(temp2,:)=[];
             break;
        end
        temp2 = temp2 + 1;
    end
    temp1 = temp1 + 1;
end

%figure;
%axis([0 100 0 100]);
%hold on;
Edges = zeros(4, length(Temp_Edge));
%Edge_X1 = zeros(1, length(Temp_Edge));
%Edge_X2 = zeros(1, length(Temp_Edge));
%Edge_Y1 = zeros(1, length(Temp_Edge));
%Edge_Y2 = zeros(1, length(Temp_Edge));
for i=1:length(Temp_Edge)
    Edges(1,i)=Voro_Vertex(Temp_Edge(i,1),1);
    Edges(2,i)=Voro_Vertex(Temp_Edge(i,2),1);
    Edges(3,i)=Voro_Vertex(Temp_Edge(i,1),2);
    Edges(4,i)=Voro_Vertex(Temp_Edge(i,2),2);
    if Is_draw==1
        plot([Edges(1,i) Edges(2,i)],[Edges(3,i) Edges(4,i)],'color',[1 .0 .0]);
    end;
end
%Edges(1, :) = Edge_X1;
%Edges(2, :) = Edge_X2;
%Edges(3, :) = Edge_Y1;
%Edges(4, :) = Edge_Y2;

Vertex = unique(Temp_Edge);
N = length(Vertex);
Vertex_count = zeros(N, 1);
for i=1:N
    Vertex_count(i) = 0;
    for j=1:length(Temp_Edge)
        if ((Temp_Edge(j,1) == Vertex(i))||(Temp_Edge(j,2) == Vertex(i)))
            Vertex_count(i) = Vertex_count(i) + 1;
        end
    end
end

Vertexes1 = [];
Vertexes3 = [];

for i=1:N
    if Vertex_count(i) == 1
        x = Voro_Vertex(Vertex(i),1);
        y = Voro_Vertex(Vertex(i),2);
        if Is_draw==1
            plot(x,y,'*','color','Green','LineWidth',2);
        end
        Vertexes1 = [Vertexes1, i];
        fprintf('1 -> point %5.0f -> x = %2.5f y = %2.5f.\n', Vertex(i), x, y);
    end
    if Vertex_count(i) == 3
        x = Voro_Vertex(Vertex(i),1);
        y = Voro_Vertex(Vertex(i),2);
        if Is_draw==1
            plot(x,y,'*','color','Red','LineWidth',2);
        end
        Vertexes3 = [Vertexes3, i];
        fprintf('3 -> point %5.0f -> x = %2.5f y = %2.5f.\n', Vertex(i), x, y);
    end
end

%% =================================================================================
% FINAL_PATH.M
% =================================================================================

%Minimum Distance
M = length(Temp_Edge);

Vertex_Cord = zeros(N, 2);
Start_distance = zeros (N, 1);
Goal_distance = zeros (N, 1);

for i=1:N
    Vertex_Cord(i,:)=Voro_Vertex(Vertex(i),:);
    Start_distance(i)=norm(Start-Vertex_Cord(i,:));
    Goal_distance(i)=norm(Goal-Vertex_Cord(i,:));
end

%weight of points
Vertex_Weight = ones(N, 1);

%figure;
%axis([0 100 0 100]);
%hold on;

AllNeirboursVertexes = cell(N, 1);

for i = 1:M
    a= find(Vertex==Temp_Edge(i,1));
    b= find(Vertex==Temp_Edge(i,2));
    AllNeirboursVertexes{a, 1} = [AllNeirboursVertexes{a, 1}, b];
    AllNeirboursVertexes{b, 1} = [AllNeirboursVertexes{b, 1}, a];
end

Curves = cell(length(Vertexes1)+2*length(Vertexes3), 1);
CurvesVertexes = zeros(size(Curves,1),2);
UsesVertexes = zeros(N,1);
Verts = zeros(length(Vertexes1)+length(Vertexes3), 2);

%create curves from 1-used vertexes to 3-used
for i=1:length(Vertexes1)
    Verts(i,1) = Voro_Vertex(Vertex(Vertexes1(i)),1);
    Verts(i,2) = Voro_Vertex(Vertex(Vertexes1(i)),2);
    curve = [];
    point = Vertexes1(i);
    curve = [curve point];
    UsesVertexes(point) = -1;
    while 1
        listNeirbourVertex = AllNeirboursVertexes{point, 1};
        if (length(listNeirbourVertex) > 2)
            UsesVertexes(point) = -3;
            break;
        end;
        for j=1:length(listNeirbourVertex)
            if (isempty(find(curve==listNeirbourVertex(j))))
                curve = [curve, listNeirbourVertex(j)];
                point = listNeirbourVertex(j);
                UsesVertexes(point) = i;
                break;
            end;
        end;
    end;    
    Curves{i,1} = curve;
    CurvesVertexes(i,1) = Vertex(curve(1));
    CurvesVertexes(i,2) = Vertex(curve(length(curve)));
    fprintf(' Curve num %5.0f with vertex %5.0f and %5.0f\n', i, Vertex(curve(1)), Vertex(curve(length(curve))));
end

%create curves from 1-used vertexes to 3-used
CurvesSize = 4; 
for i=1:length(Vertexes3)
    Verts(4+i,1) = Voro_Vertex(Vertex(Vertexes3(i)),1);
    Verts(4+i,2) = Voro_Vertex(Vertex(Vertexes3(i)),2);
    pointbegin = Vertexes3(i);        
    listNeirbourVertex = AllNeirboursVertexes{pointbegin, 1};
    for k=1:length(listNeirbourVertex)
        curve = [];
        curve = [curve pointbegin];
        UsesVertexes(pointbegin) = -3;
        if (UsesVertexes(listNeirbourVertex(k)) == 0)
            pointnext = listNeirbourVertex(k);        
            curve = [curve pointnext];
            UsesVertexes(pointnext) = CurvesSize + 1;
            while 1
                listNeirbourVertexTemp = AllNeirboursVertexes{pointnext, 1};
                if (length(listNeirbourVertexTemp) > 2)
                    UsesVertexes(pointbegin) = -3;
                    break;
                end;
                for j=1:length(listNeirbourVertexTemp)
                    if (isempty(find(curve==listNeirbourVertexTemp(j),1)))
                        curve = [curve, listNeirbourVertexTemp(j)];
                        pointnext = listNeirbourVertexTemp(j);
                        UsesVertexes(pointnext) = CurvesSize + 1;
                        break;
                    end;
                end;
            end;
            CurvesSize = CurvesSize + 1;
            Curves{CurvesSize,1} = curve;
            CurvesVertexes(CurvesSize,1) = Vertex(curve(1));
            CurvesVertexes(CurvesSize,2) = Vertex(curve(length(curve)));
            fprintf(' Curve num %5.0f with vertex %5.0f and %5.0f\n', CurvesSize, Vertex(curve(1)), Vertex(curve(length(curve))));
        end
    end
end

VertWithoutCurve = cell(CurvesSize, 1);
PathWithoutCurve = cell(CurvesSize, 1);
CostWithoutCurve = [];

%first path without delete
Voro_Graph = inf*ones(N);
Goal_distance(:,1) = inf;
Start_distance(:,1) = inf;
for i = 1:M
    a= find(Vertex==Temp_Edge(i,1));
    b= find(Vertex==Temp_Edge(i,2));    
    if (UsesVertexes(a)>0 && UsesVertexes(a)<5) || (UsesVertexes(b)>0 && UsesVertexes(b)<5)
        continue;
    end;
    Distance = norm(Vertex_Cord(a,:)-Vertex_Cord(b,:));
    Voro_Graph(a,b)=Distance*Vertex_Weight(b);
    Voro_Graph(b,a)=Distance*Vertex_Weight(a);
    Start_distance(a)=norm(Start-Vertex_Cord(a,:));
    Start_distance(b)=norm(Start-Vertex_Cord(b,:));
    Goal_distance(a)=norm(Goal-Vertex_Cord(a,:));
    Goal_distance(b)=norm(Goal-Vertex_Cord(b,:));
end
[Dummy Index_Start]=min(Start_distance);
[Dummy Index_Goal]=min(Goal_distance);
[path, cost] = dijkstra(Voro_Graph,Index_Start,Index_Goal);

x=[Start(1) Vertex_Cord(Index_Start,1)];
y=[Start(2) Vertex_Cord(Index_Start,2)];
lineStart = [x(1,1) y(1,1) x(1,2) y(1,2)];
x=[Vertex_Cord(Index_Goal,1) Goal(1)];
y=[Vertex_Cord(Index_Goal,2) Goal(2)];
lineGoal = [x(1,1) y(1,1) x(1,2) y(1,2)];
cost = cost + sqrt((lineStart(1,3)-lineStart(1,1))^2 + (lineStart(1,4)-lineStart(1,2))^2);
cost = cost + sqrt((lineGoal(1,3) -lineGoal(1,1))^2 +  (lineGoal(1,4)- lineGoal(1,2))^2);

PathWithoutCurve{1,1} = path;
CostWithoutCurve = [CostWithoutCurve, cost];

CurvesInBeginPath = [];
cnt = 0;
fprintf(' Path with vertex');
for k=1:length(path)
    curv = UsesVertexes(path(1,k));
    if curv>0 
        if isempty(CurvesInBeginPath)
            cnt = cnt + 1;
            CurvesInBeginPath = [CurvesInBeginPath, curv];
        else
            if CurvesInBeginPath(cnt)~=curv
                cnt = cnt + 1;
                CurvesInBeginPath = [CurvesInBeginPath, curv];
            end;
        end;
    else
        VertWithoutCurve{1,1} = [VertWithoutCurve{1,1} path(1,k)];
        fprintf(' %5.0f', Vertex(path(1,k)));
        if k<length(path)
            if UsesVertexes(path(1,k+1))>0
                continue;
            end;
            for i=1:CurvesSize
                if (CurvesVertexes(i,1)==Vertex(path(1,k)) && CurvesVertexes(i,2)==Vertex(path(1,k+1))) || (CurvesVertexes(i,2)==Vertex(path(1,k)) && CurvesVertexes(i,1)==Vertex(path(1,k+1)))
                    if CurvesInBeginPath(cnt)~=i
                        cnt = cnt + 1;
                        CurvesInBeginPath = [CurvesInBeginPath, i];
                    end;
                    break;
                end;
            end;
        end;
    end;
end;
fprintf('\n');

fprintf(' Path with curve');
for i=1:size(CurvesInBeginPath,2)
    fprintf(' %5.0f', CurvesInBeginPath(1,i))
end;
fprintf('\n');

%{
combineCurves = cell(CurvesSize, CurvesSize);
tempCombine1 = sort(CurvesInBeginPath);
for i=1:length(CurvesInBeginPath)
    combineCurves{1,i} = tempCombine1(i);
end;

number = 2;
curves_delete = 5:1:CurvesSize;
for combinations = 1:2
    Comb = nchoosek(curves_delete, combinations);
    count_added = 0;
    for c=1:size(Comb, 1)
        
        %k=0;
        %for s=1:combinations
        %    if ~isempty(find(Comb(c,s)==CurvesInBeginPath, 1))
        %        k = k+1;
        %    end;
        %end;
        %if k==0
        %    continue;
        %end;
        if combinations==1 && isempty(find(Comb(c,:)==CurvesInBeginPath, 1))
            continue;
        end;
        if combinations>1
            find_comb = 0;
            for j=1:size(combineCurves(combinations-1,:), 2)
                combine=combineCurves{combinations-1,j};
                if isempty(combine)
                    break;
                end;
                comb = 0;
                for k=1:length(combine)
                    if isempty(find(Comb(c,:)==combine(k), 1))
                        break;
                    end;
                    comb = comb+1;
                end;
                if comb==combinations-1
                    find_comb = 1;
                    break;
                end;
            end;
            if find_comb==0
                continue;
            end;
        end;            
        
        Voro_Graph = inf*ones(N);
        Goal_distance(:,1) = inf;
        Start_distance(:,1) = inf;

        for i = 1:M
            a= find(Vertex==Temp_Edge(i,1));
            b= find(Vertex==Temp_Edge(i,2));
            if (UsesVertexes(a)>0 && UsesVertexes(a)<5) || (UsesVertexes(b)>0 && UsesVertexes(b)<5)
                continue;
            end;
            %x=[Vertex_Cord(a,1) Vertex_Cord(b,1)];
            %y=[Vertex_Cord(a,2) Vertex_Cord(b,2)];

            flag = 0;
            for l=1:combinations
                num = Comb(c,l);
                if UsesVertexes(a)<0 && UsesVertexes(b)<0
                    if (CurvesVertexes(num,1)==Vertex(a) && CurvesVertexes(num,2)==Vertex(b)) || (CurvesVertexes(num,2)==Vertex(a) && CurvesVertexes(num,1)==Vertex(b))
                        flag = 1;
                        break;
                    end;
                end;
                if UsesVertexes(a)==num || UsesVertexes(b)==num
                    flag = 1;
                    break;
                end;
            end;
            
            if (flag==0)
                Distance = norm(Vertex_Cord(a,:)-Vertex_Cord(b,:));
                Voro_Graph(a,b)=Distance*Vertex_Weight(b);
                Voro_Graph(b,a)=Distance*Vertex_Weight(a);

                Start_distance(a)=norm(Start-Vertex_Cord(a,:));
                Start_distance(b)=norm(Start-Vertex_Cord(b,:));
                Goal_distance(a)=norm(Goal-Vertex_Cord(a,:));
                Goal_distance(b)=norm(Goal-Vertex_Cord(b,:));

            %    plot(x,y,'color','Green','LineWidth',2);
            end;
        end

        %for i=1:N
        %    Start_distance(i)=norm(Start-Vertex_Cord(i,:));
        %    Goal_distance(i)=norm(Goal-Vertex_Cord(i,:));
        %end

        [Dummy Index_Start]=min(Start_distance);
        [Dummy Index_Goal]=min(Goal_distance);
        
        x=[Start(1) Vertex_Cord(Index_Start,1)];
        y=[Start(2) Vertex_Cord(Index_Start,2)];
        lineStart = [x(1,1) y(1,1) x(1,2) y(1,2)];
        x=[Vertex_Cord(Index_Goal,1) Goal(1)];
        y=[Vertex_Cord(Index_Goal,2) Goal(2)];
        lineGoal = [x(1,1) y(1,1) x(1,2) y(1,2)];
        findIntersect = 0;
        for i=2:Num_Object
            for r=1:length(X1{i})
               a=r;
               if(r==length(X1{i}))
                   b=1;
               else
                   b=r+1;
               end
               line_edge = [X1{i}(a,1) X1{i}(a,2) X1{i}(b,1) X1{i}(b,2)];
               intersection_point = intersectEdges(lineStart, line_edge);
               if ~isnan(intersection_point(1,1)) || ~isnan(intersection_point(1,2))
                   findIntersect = 1;
                   break;
               end
               intersection_point = intersectEdges(lineGoal, line_edge);
               if ~isnan(intersection_point(1,1)) || ~isnan(intersection_point(1,2))
                   findIntersect = 1;
                   break;
               end
            end
            if findIntersect==1
                break;
            end;
        end
        if findIntersect==1
            continue;
        end
        
        [path, cost] = dijkstra(Voro_Graph,Index_Start,Index_Goal);
        cost = cost + sqrt((lineStart(1,3)-lineStart(1,1))^2 + (lineStart(1,4)-lineStart(1,2))^2);
        cost = cost + sqrt((lineGoal(1,3) -lineGoal(1,1))^2 +  (lineGoal(1,4)- lineGoal(1,2))^2);

        %for i=1:length(path)-1
        %    x=[Vertex_Cord(path(i),1) Vertex_Cord(path(i+1),1)];
        %    y=[Vertex_Cord(path(i),2) Vertex_Cord(path(i+1),2)];
        %    plot(x,y,'-','color','r','LineWidth',2);
        %end
        
        findpath = 0;
        for i=1:length(CostWithoutCurve)
            if cost==CostWithoutCurve(i) || isinf(cost)
                findpath = 1;
                break;
            end
        end
        if (findpath==0)
            PathWithoutCurve{number,1} = path;
            CostWithoutCurve = [CostWithoutCurve, cost];
            for i=1:combinations
                fprintf(' %5.0f', Comb(c,i));
            end;
            fprintf(' Added with cost %3.5f\n', cost);
            count_added = count_added + 1;
            if combinations>1
                combineCurves{combinations,count_added} = Comb(c,:);
            end;
            
            VertWithoutCurve{number,1} = [];
            fprintf(' Path with vertex');
            for k=1:length(path)
                curv = UsesVertexes(path(1,k));
                if curv>0 
                else
                    VertWithoutCurve{number,1} = [VertWithoutCurve{number,1} path(1,k)];
                    fprintf(' %5.0f', Vertex(path(1,k)));
                    
                end;
            end;
            fprintf('\n');
            
            number = number + 1;
            
        else
            for i=1:combinations
                fprintf(' %5.0f', Comb(c,i));
            end;
            fprintf(' do not added\n');
        end;
        %break;%!!!!!!!!!
        if number == 3
            break;
        end
    end
end

for i=number:CurvesSize
    PathWithoutCurve(number) = [];
    VertWithoutCurve{number,1} = [];
end;

s = length(Vertex_Cord)+1;
g = length(Vertex_Cord)+2;
Vertex_Cord(s, 1) = Start(1,1);
Vertex_Cord(s, 2) = Start(1,2);
Vertex_Cord(g, 1) = Goal(1,1);
Vertex_Cord(g, 2) = Goal(1,2);
for i=1:length(PathWithoutCurve)
    PathWithoutCurve{i,1} = [s PathWithoutCurve{i,1}];
    PathWithoutCurve{i,1} = [PathWithoutCurve{i,1} g];
end;

[Dummy MinCost]=min(CostWithoutCurve);
path = PathWithoutCurve{2,1};
fprintf(' DV with min cost is %5.0f\n', MinCost);
%}

%{
for i=1:Num_Object
    for r=1:length(X1{i})
       a=r;
       if(r==length(X1{i}))
           b=1;
       else
           b=r+1;
       end
       x=[X1{i}(a,1) X1{i}(b,1)];
       y=[X1{i}(a,2) X1{i}(b,2)];
       if Is_draw==1
           plot(x,y,'g');
           hold on;
       end;
    end
end
if Is_draw==1
    drawnow;
end;

if Is_draw==1
    plot(Start(1),Start(2),'pw','Markersize',13, 'Color', 'g');
    plot(Goal(1),Goal(2), 'pw','Markersize',13, 'Color', 'b');
end;
 %figure(1);
 %axis([0 100 0 100]);
hold on;

 for i=1:length(Temp_Edge)
    Edge_X1(i)=Voro_Vertex(Temp_Edge(i,1),1);
    Edge_X2(i)=Voro_Vertex(Temp_Edge(i,2),1);
    Edge_Y1(i)=Voro_Vertex(Temp_Edge(i,1),2);
    Edge_Y2(i)=Voro_Vertex(Temp_Edge(i,2),2);
    if Is_draw==1
        plot([Edge_X1(i) Edge_X2(i)],[Edge_Y1(i) Edge_Y2(i)],'color',[.7 .7 .7],'LineWidth',2);
    end;
end
%}
 
 %x=[Start(1) Vertex_Cord(path(1),1)];
 %y=[Start(2) Vertex_Cord(path(1),2)];
 %if Is_draw==1
 %   plot(x,y,'-','color','r','LineWidth',2);
 %   drawnow;
 %end;
 
 %traj = [traj;[x', y']];
 %aux_x = traj(end-1,1);
 %aux_y = traj(end-1,2);
 %traj(end-1,1) = traj(end-2,1);
 %traj(end-1,2) = traj(end-2,2);
 %traj(end-2,1) = aux_x;
 %traj(end-2,2) = aux_y;
 
 
 for i=1:length(path)-1
     x=[Vertex_Cord(path(i),1) Vertex_Cord(path(i+1),1)];
     y=[Vertex_Cord(path(i),2) Vertex_Cord(path(i+1),2)];
     if Is_draw==1
         plot(x,y,'-','color','r','LineWidth',2);
         %drawnow;
         hold on;
     end;
     traj = [traj;[x', y']];
 end
 
 traj = [Start; traj];
 traj = [traj; Goal];
 
 %x=[Vertex_Cord(path(i+1),1) Goal(1)];
 %y=[Vertex_Cord(path(i+1),2) Goal(2)];
 %if Is_draw==1
 %    plot(x,y,'-','color','r','LineWidth',2);
 %    drawnow;
 %end;
 %traj = [traj;[x', y']];
 

end%function

%% Dijkstra function
function [shortestPath, totalCost] = dijkstra(netCostMatrix, s, d)
%==============================================================
% shortestPath: the list of nodes in the shortestPath from source to destination;
% totalCost: the total cost of the  shortestPath;
% farthestNode: the farthest node to reach for each node after performing the routing;
% n: the number of nodes in the network;
% s: source node index;
% d: destination node index;
%==============================================================
%  Code by:
% ++by Xiaodong Wang
% ++23 Jul 2004 (Updated 29 Jul 2004)
% ++http://www.mathworks.com/matlabcentral/fileexchange/5550-dijkstra-shortest-path-routing
% Modifications (simplifications) by Meral Shirazipour 9 Dec 2009
%==============================================================
n = size(netCostMatrix,1);
for i = 1:n
    % initialize the farthest node to be itself;
    farthestPrevHop(i) = i; % used to compute the RTS/CTS range;
    farthestNextHop(i) = i;
end

% all the nodes are un-visited;
visited(1:n) = false;

distance(1:n) = inf;    % it stores the shortest distance between each node and the source node;
parent(1:n) = 0;

distance(s) = 0;
for i = 1:(n-1),
    temp = [];
    for h = 1:n,
         if ~visited(h)  % in the tree;
             temp=[temp distance(h)];
         else
             temp=[temp inf];
         end
     end;
     [t, u] = min(temp);      % it starts from node with the shortest distance to the source;
     visited(u) = true;         % mark it as visited;
     for v = 1:n,                % for each neighbors of node u;
         if ( ( netCostMatrix(u, v) + distance(u)) < distance(v) )
             distance(v) = distance(u) + netCostMatrix(u, v);   % update the shortest distance when a shorter shortestPath is found;
             parent(v) = u;     % update its parent;
         end;             
     end;
end;

shortestPath = [];
if parent(d) ~= 0   % if there is a shortestPath!
    t = d;
    shortestPath = [d];
    while t ~= s
        p = parent(t);
        shortestPath = [p shortestPath];
        
        if netCostMatrix(t, farthestPrevHop(t)) < netCostMatrix(t, p)
            farthestPrevHop(t) = p;
        end;
        if netCostMatrix(p, farthestNextHop(p)) < netCostMatrix(p, t)
            farthestNextHop(p) = t;
        end;

        t = p;      
    end;
end;

totalCost = distance(d);
%return;

end %dijkstra
