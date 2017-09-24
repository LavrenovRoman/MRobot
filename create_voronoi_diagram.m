function [ Vertex, Voro_Vertex, Temp_Edge, UsesVertexes, Edges, Verts, CurvesSize, CurvesVertexes] = create_voronoi_diagram( limits, ...
    Num_Object, X_Total_points, Y_Total_points, All_cells_Number, Cell_start, Is_draw)
%CREATE Summary of this function goes here
%   Detailed explanation goes here
%clear all;
%close all;
%clc;

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

AllNeirboursVertexes = cell(N, 1);

%Minimum Distance
M = length(Temp_Edge);

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

end

