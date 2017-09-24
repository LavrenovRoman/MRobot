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


function [traj, Vertex_Cord, PathWithoutCurve, CostWithoutCurve, VertWithoutCurve] = rmt_get_voronoi( Vertex, ...
    Voro_Vertex, CurvesSize, Temp_Edge, UsesVertexes, CurvesVertexes, Num_Object, Start, Goal, X1, Is_draw)

%% =================================================================================
% FINAL_PATH.M
% =================================================================================
traj = [];

N = length(Vertex);
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
