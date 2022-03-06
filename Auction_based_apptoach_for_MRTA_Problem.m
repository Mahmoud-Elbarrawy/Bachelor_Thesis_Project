%%Description
% cluster Market-based MTSP ---> TSP 2-opt
clc;
clear all;
%https://link.springer.com/content/pdf/10.1007%2Fs10846-012-9805-3.pdf

%% creat matrix of nods
A=100; % size of matrix
[nod_list_x_y,nod_matrix]=matrix_into_nod(A);
%creat map occupancy grid with obsitcals
[map,grid_matrix]=creat_occ_map(A);
show(map)
hold on
% Create G shortest path
[G] =G_creat(map,nod_matrix,grid_matrix);



%% Problem Parameters

% n_tasks=length(tasksposM); 
% n_agents=length(agentsposM);

n_tasks=30; 
n_agents=7;
for i=1:n_tasks
tasks(i).ID=i;  % Task idx
tasks(i).t=5; % Time required to finish  
tasks(i).w=5;   % Weights in this task to carry 
x=(ceil(rand*A))-0.5;
y=(ceil(rand*A))-0.5;
z=0;
while getOccupancy(map,[x y])
x=(ceil(rand*A))-0.5;
y=(ceil(rand*A))-0.5;
end 
% x=tasksposM(i,1);
% y=tasksposM(i,1);
tasks(i).pos=[x;y];
tasks(i).nod=0;
tasks(i).cluster=0;

end

tasksposM= [tasks(1,:).pos]';

for i=1:n_agents
agents(i).ID=i; %Indx
agents(i).v=5;  % Max velocity in m/s
agents(i).EL=100;   % Electric charge level
agents(i).w=5;  % Max weight to carry in Kg
agents(i).current_pos = n_tasks+i;  % Position in mape (random) 

x=((ceil(rand*A))-0.5);
y=((ceil(rand*A))-0.5);
z=0;
while getOccupancy(map,[x y])
x=((ceil(rand*A))-0.5);
y=((ceil(rand*A))-0.5);
end 
% x=agentsposM(i,1);
% y=agentsposM(i,2);
agents(i).pos=[x;y];
agents(i).assgintask=0;
agents(i).cost=0;
agents(i).nod=0;
agents(i).nod_path_x_y=0;
end
agentsposM = [agents(1,:).pos]';
%% creat a nod position for each agent and task to make path planing
 for i=1:n_agents
   nod_numm = find(nod_list_x_y(2,:)==agentsposM(i,1)& nod_list_x_y(3,:)==agentsposM(i,2));
   agents(i).nod=nod_numm;
 end
 for i=1:n_tasks
   nod_numm = find(nod_list_x_y(2,:)==tasksposM(i,1)& nod_list_x_y(3,:)==tasksposM(i,2));  
   tasks(i).nod=nod_numm;
 end
 %% modling matrax distances btween each taks and onther and robot
 distances =zeros(n_tasks+n_agents,n_tasks+n_agents);

for i=1:n_tasks
    for j=1:n_tasks+n_agents
       if(j<=n_tasks) 
      [path1,d]=shortestpath(G,tasks(i).nod,tasks(j).nod);
      distances(i,j)=d;
       else
           [path1,d]=shortestpath(G,tasks(i).nod,agents(j-n_tasks).nod);
            distances(i,j)=d;
       end  
    end
end
for i=1:n_agents
    for j=1:n_tasks+n_agents
        if(j<=n_tasks)
      [path1,d]=shortestpath(G,agents(i).nod,tasks(j).nod);
      distances(i+n_tasks,j)=d;
        else
           [path1,d]=shortestpath(G,agents(i).nod,agents(j-n_tasks).nod);
           distances(i+n_tasks,j)=d; 
        end
    end
end


%% K-means method via 2-opt
tic
if (n_agents<n_tasks)
[cluster_indx,Clusters_pos] = kmeans(tasksposM,n_agents);
for i=1:n_tasks
  tasks(i).cluster= cluster_indx(i,1);   
end
else
    [cluster_indx,Clusters_pos] = kmeans(tasksposM,n_tasks);
for i=1:n_tasks
  tasks(i).cluster= cluster_indx(i,1);   
end
end
 
%% creat a nod for each clusters 
% clusters_nod=zeros(n_agents,3);
% for i=1:n_agents
%    nod_numm = find(nod_list_x_y(2,:)==((ceil(Clusters_pos(i,1)))-0.5)& nod_list_x_y(3,:)==((ceil(Clusters_pos(i,2)))-0.5));
%    clusters_nod(i,1)=((ceil(Clusters_pos(i,1)))-0.5);
%    clusters_nod(i,2)=((ceil(Clusters_pos(i,2)))-0.5);
%    clusters_nod(i,3)=nod_numm;
% end
%% creat distances_cluster matrix and robots
% distances_cluster=zeros(2*n_agents,2*n_agents);
% for i=1:n_agents
%     for j=1:2*n_agents
%        if (j<=n_agents)
%          [path1,d]=shortestpath(G,clusters_nod(i,3),clusters_nod(j,3));
%           distances_cluster(i,j)=d;
%        else
%          [path1,d]=shortestpath(G,clusters_nod(i,3),agents(j-n_agents).nod);
%           distances_cluster(i,j)=d;  
%        end
%        
%     end
% end
% for i=1:n_agents
%     for j=1:2*n_agents
%        if (j<=n_agents)
%          [path1,d]=shortestpath(G,agents(i).nod,clusters_nod(j,3));
%           distances_cluster(i+n_agents,j)=d;
%        else
%          [path1,d]=shortestpath(G,agents(i).nod,agents(j-n_agents).nod);
%           distances_cluster(i+n_agents,j)=d;  
%        end
%        
%     end
% end
%% Auction
available_agents=agents;
%creat groups of clustares 
 for i=1:n_agents
   IDS=find(cluster_indx==i);
   Group(i).available_clusters=IDS;
   Group(i).new_order=[];
   Group(i).cost=inf;
   Group(i).winner=[];
 end
 % loop on each group of clusters
 Group_count=1;
while(Group_count<=length(Group))
% loop on each agent for each Group    
 agents_count=1;
 while(agents_count<=length(available_agents))
    
 M=zeros(length(Group(Group_count).available_clusters)+1,length(Group(Group_count).available_clusters)+1);
 for i=1:(length(Group(Group_count).available_clusters))
     for j=1:(length(Group(Group_count).available_clusters)+1)
        if (j<=length(Group(Group_count).available_clusters))
         % [path1,d]=shortestpath(G,tasks(Group(Group_count).available_clusters(i)).nod,tasks(Group(Group_count).available_clusters(j)).nod);
          %M(i,j)=d;
          M(i,j)=distances((Group(Group_count).available_clusters(i)),(Group(Group_count).available_clusters(j))); 
           
        else
%           [path1,d]=shortestpath(G,tasks(Group(Group_count).available_clusters(i)).nod,available_agents(agents_count).nod);
%           M(i,j)=d;
          M(i,j)=distances((Group(Group_count).available_clusters(i)),(n_tasks+(available_agents(agents_count).ID)));
        end
        
     end 
 end

 for j=1:(length(Group(Group_count).available_clusters)+1)
        if (j<=(length(Group(Group_count).available_clusters)))
%           [path1,d]=shortestpath(G,available_agents(agents_count).nod,tasks(Group(Group_count).available_clusters(j)).nod);
%            M((length(Group(Group_count).available_clusters)+1),j)=d;
           M((length(Group(Group_count).available_clusters)+1),j)=distances((n_tasks+available_agents(agents_count).ID),(Group(Group_count).available_clusters(j)));
        else
%           [path1,d]=shortestpath(G,available_agents(agents_count).nod,available_agents(agents_count).nod);
%            M((length(Group(Group_count).available_clusters)+1),j)=d;
           M((length(Group(Group_count).available_clusters)+1),j)=distances((n_tasks+available_agents(agents_count).ID),(n_tasks+available_agents(agents_count).ID));

        end
        
  end


[order,cost] = tsspsearch(M,1);
%check cost
temp=cost;
 if(Group(Group_count).cost>temp)
  Group(Group_count).new_order=[];
  Group(Group_count).cost=temp;
  Group(Group_count).winner=available_agents(agents_count).ID;
%save the new order
  for N=2:length(order)
  Group(Group_count).new_order=cat(2,Group(Group_count).new_order,Group(Group_count).available_clusters(order(N)));
  end
 end
 agents_count=agents_count+1;
 end
 
 % update available agents
 for i=1:length(available_agents)
 if(available_agents(i).ID==Group(Group_count).winner)
  temp_indx_of_ID=i;
 end
 end
   available_agents(temp_indx_of_ID)=[];  
   Group_count=Group_count+1;
end
%% assin tasks for the agents
for i=1:length(Group)
agents(Group(i).winner).assgintask=Group(i).new_order;
agents(Group(i).winner).cost=Group(i).cost;
end
toc;
    %% %% Figure construction
colors = ['r';'b';'y';'g';'c';'m';'r';'b';'y';'g';'c';'m';'r';'b';'y';'g';'c';'m';'r';'b';'y';'g';'c';'m';'r';'b';'y';'g';'c';'m';'r';'b';'y';'g';'c';'m';'r';'b';'y';'g';'c';'m'];
%figure('units','normalized','position',[0.05 0.2 0.9 0.6]);
% gscatter(XGrid(:,1),XGrid(:,2),idx,...
%     [0,0.75,0.75;0.75,0,0.75;0.75,0.75,0],'..');
%hold on;
%subplot(1,1,1);
cc = 1;
for i = 1:n_tasks
    if cc == 6
        cc = 1;
    end
    plot(tasksposM(i,1),tasksposM(i,2),'r.', 'color', colors(cluster_indx(i,1)), 'MarkerSize', 18);
%     plot(tasksposM(i,1),tasksposM(i,2),i, 'color', colors(cluster_indx(i,1)), 'MarkerSize', 18);

end

% for ac = 1:n_agents
%     temp_sim(ac,:) = [agents(ac).pos(1,1) agents(ac).pos(1,2)];
%     plot(agents(ac).pos(1,1), agents(ac).pos(1,2), 'x');
%     plot(agentsposM(ac,1),agentsposM(ac,2),'o');
%     %plot([agentsposM(ac,1); tasksposM(agents(ac).assgintask,1)], [agentsposM(ac,2); tasksposM(agents(ac).assgintask,2)], 'r-');
% end
%  plot(agents(1).nod_path_x_y(2,:),agents(1).nod_path_x_y(3,:), 'r-');

xlabel('x_axis')
ylabel('y_axis')
title('MTSP')
 
xlim([0 A]);
ylim([0 A]);

 %% creat x_y set of point in order for each robot 
 for j=1:n_agents
     if( ~isempty(agents(j).assgintask))
    [path1,d]=shortestpath(G,agents(j).nod,tasks(agents(j).assgintask(1)).nod);
     [nod_path_x_y]= way(path1,nod_list_x_y);
     agents(j).nod_path_x_y=nod_path_x_y;

  for i=2:length(agents(j).assgintask)
      [path1,d]=shortestpath(G,tasks(agents(j).assgintask(i-1)).nod,tasks(agents(j).assgintask(i)).nod);
      [nod_path_x_y]= way(path1,nod_list_x_y);
      nod_path_x_y(:,1)=[];
     agents(j).nod_path_x_y=cat(2,agents(j).nod_path_x_y,nod_path_x_y);
  
  end
     end
 end
 
 for i=1:n_agents
if(agents(i).nod_path_x_y~=0)
plot(agents(i).nod_path_x_y(2,:),agents(i).nod_path_x_y(3,:), 'r-','color',colors(1));
       plot(agents(i).nod_path_x_y(2,1),agents(i).nod_path_x_y(3,1), 'o');

else
     plot(agents(i).pos(1),agents(i).pos(2), 'o');
end
end
      minmax_cost=max([agents.cost]);
total_cost=0;
 for i=1:n_agents
    total_cost=total_cost+agents(i).cost; 
 end
 
%  
%% Robots
 % number of robots
% Robots_LocationX =[agents(1).nod_path_x_y(3,2) agents(1).nod_path_x_y(2,1)]; 
Robots_Location=zeros(n_agents,2);
for i=1:n_agents
    if(~isempty(agents(i).assgintask))
Robots_Location(i,:) =[agents(i).nod_path_x_y(2,1) agents(i).nod_path_x_y(3,1)];
    end
 end
Robots = cell(n_agents,1);
for i=1:n_agents
    Robots{i,1}.location = Robots_Location(i,:);
    Robots{i,1}.controller = robotics.PurePursuit;
    Robots{i,1}.controller.LookaheadDistance = 0.5;
    Robots{i,1}.controller.MaxAngularVelocity = 100;
    Robots{i,1}.controller.DesiredLinearVelocity = 1;
    Robots{i,1}.color = colors(i);
    Robots{i,1}.size = 1;
    Robots{i,1}.velocity = 0;
    Robots{i,1}.omega = 0;
    Robots{i,1}.initalpose = [Robots_Location(i,:) 0];
    Robots{i,1}.simulation = MyRobot([Robots{i,1}.initalpose], Robots{i,1}.color, Robots{i,1}.size);
hold on;
end

for i=1:n_agents
    if(~isempty(agents(i).assgintask))
Robots{i,1}.controller.Waypoints =[agents(i).nod_path_x_y(2,:)' agents(i).nod_path_x_y(3,:)'];
%Robots{2,1}.controller.Waypoints = p2;

    end
end
%% Simulation play
while true
    for i=1:n_agents
      if(~isempty(agents(i).assgintask))  
    [v1,w1] = step(Robots{i,1}.controller, Robots{i,1}.simulation.CurrentPose);
    if norm(Robots{i,1}.simulation.CurrentPose(1,1:2) - Robots{i,1}.controller.Waypoints(end,:)) < 0.2
        drive(Robots{i,1}.simulation, 0, 0);
    else
        drive(Robots{i,1}.simulation, v1, w1);
    end
      end
    end
%     [v2,w2] = step(Robots{2,1}.controller, Robots{2,1}.simulation.CurrentPose);
%     if norm(Robots{2,1}.simulation.CurrentPose(1,1:2) - Robots{2,1}.controller.Waypoints(end,:)) < 0.5
%         drive(Robots{2,1}.simulation, 0, 0);
%     else
%         drive(Robots{2,1}.simulation, v2, w2);
%    end
end

%% Create nod_matrix
function [nod_list_x_y,nod_matrix] = matrix_into_nod(A)

nod_matrix=zeros(A,A);
nod_counter=1;
x_nod=[];
y_nod=[];
nod_num=[];
q=size(nod_matrix,1)+1;
for a=1:size(nod_matrix)
    for b=1:size(nod_matrix)
        if nod_matrix(a,b)==0
            nod_matrix(a,b)=nod_counter;
            x_nod=cat(2,x_nod,b-0.5);
            y_nod=cat(2,y_nod,q-a-0.5);
            nod_num=cat(2,nod_num,nod_counter);
           nod_counter=nod_counter+1;
        end
    end
end
nod_list_x_y=[nod_num;x_nod;y_nod];
end
%% crear_occ_map
function [map,grid_matrix] = creat_occ_map(A)

  grid_matrix = zeros(A,A);
 grid_matrix(2,2)=1;
    grid_matrix(1,:) = 1; grid_matrix(:,1) = 1; grid_matrix(100,:) = 1;
    grid_matrix(:,100) = 1; 
    grid_matrix(40:65, 20:22) = 1;  grid_matrix(80:100, 20:22) = 1; 
    grid_matrix(32:33, 21:40) = 1;
     grid_matrix(1:20, 40:42) = 1; grid_matrix(40:65, 40:42) = 1;
     grid_matrix(100-38:100-17, 30:31) = 1; grid_matrix(60:100, 75+8:76+8) = 1;
     grid_matrix(32:33, 70:100) = 1; grid_matrix(23:40, 54) = 1;

map = robotics.BinaryOccupancyGrid(grid_matrix, 1);
end
%% creat G graph
function [G] =G_creat(map,nod_matrix,grid_matrix)

s=[];
t=[];

for i=1:size(grid_matrix,1)
    for j=1:size(grid_matrix,1)
       
         if (getOccupancy(map,[i j],'grid')==0 && (i-1)~=0 && (i-1~=size(grid_matrix,1)+1) && (j)~=0 && (j~=size(grid_matrix,1)+1))
           if getOccupancy(map,[i-1 j],'grid')==0
            s=cat(2,s,nod_matrix(i,j));
            t=cat(2,t,nod_matrix(i-1,j));
           end
          end

          if (getOccupancy(map,[i j],'grid')==0 && (i)~=0 && (i~=size(grid_matrix,1)+1) && (j-1)~=0 && (j-1~=size(grid_matrix,1)+1))
           if getOccupancy(map,[i j-1],'grid')==0
            s=cat(2,s,nod_matrix(i,j));
            t=cat(2,t,nod_matrix(i,j-1));
           end
          end
           if (getOccupancy(map,[i j],'grid')==0 && (i)~=0 && (i~=size(grid_matrix,1)+1) && (j+1)~=0 && (j+1~=size(grid_matrix,1)+1))
           if getOccupancy(map,[i j+1],'grid')==0
            s=cat(2,s,nod_matrix(i,j));
            t=cat(2,t,nod_matrix(i,j+1));
           end
           end
      if (getOccupancy(map,[i j],'grid')==0 && (i+1)~=0 && (i+1~=size(grid_matrix,1)+1) && (j)~=0 && (j~=size(grid_matrix,1)+1))
           if getOccupancy(map,[i+1 j],'grid')==0
            s=cat(2,s,nod_matrix(i,j));
            t=cat(2,t,nod_matrix(i+1,j));
           end
      end
%edg path planign 
%           if (getOccupancy(map,[i j],'grid')==0 && (i-1)~=0 && (i-1~=size(grid_matrix,1)+1) && (j+1)~=0 && (j+1~=size(grid_matrix,1)+1))
%            if getOccupancy(map,[i-1 j+1],'grid')==0
%             s=cat(2,s,nod_matrix(i,j));
%             t=cat(2,t,nod_matrix(i-1,j+1));
%            end
%           end
%            if (getOccupancy(map,[i j],'grid')==0 && (i+1)~=0 && (i+1~=size(grid_matrix,1)+1) && (j-1)~=0 && (j-1~=size(grid_matrix,1)+1))
%            if getOccupancy(map,[i+1 j-1],'grid')==0
%             s=cat(2,s,nod_matrix(i,j));
%             t=cat(2,t,nod_matrix(i+1,j-1));
%            end
%            end
%      
%         if (getOccupancy(map,[i j],'grid')==0 && (i-1)~=0 && (i-1~=size(grid_matrix,1)+1) && (j-1)~=0 && (j-1~=size(grid_matrix,1)+1))
%           if getOccupancy(map,[i-1 j-1],'grid')==0
%             s=cat(2,s,nod_matrix(i,j));
%             t=cat(2,t,nod_matrix(i-1,j-1));
%           end
%        end
%       
%            if(getOccupancy(map,[i j],'grid')==0 && (i+1)~=0 && (i+1~=size(grid_matrix,1)+1) && (j+1)~=0 && (j+1~=size(grid_matrix,1)+1))
%            if getOccupancy(map,[i+1 j+1],'grid')==0
%             s=cat(2,s,nod_matrix(i,j));
%             t=cat(2,t,nod_matrix(i+1,j+1));
%            end
%           end
     end
 end    
G = graph(s,t);
end
%% creat a 3*N path of nodest between start and target (x,y)
function [nod_path_x_y]= way(path1,nod_list_x_y)
nod_path_x_y=[];
nod_path=[];
nod_path_x=[];
 nod_path_y=[];
 for j=1:length(path1)
  for i=1:length(nod_list_x_y)
    if nod_list_x_y(1,i)==path1(1,j)
        nod_path=cat(2,nod_path,nod_list_x_y(1,i));
        nod_path_x=cat(2,nod_path_x,nod_list_x_y(2,i));
        nod_path_y=cat(2,nod_path_y,nod_list_x_y(3,i));
        nod_path_x_y=[nod_path;nod_path_x; nod_path_y];
    end    
  end
 end
end
%% TSP solvaer (tsspsearch)
function [p,L] = tsspsearch(X,m)
%TSPSEARCH Heuristic method for Traveling Salesman Problem (TSP).
%   [P,L] = TSPSEARCH(X,M) gives a tour P of length L. X is either a
%   coordinate matrix of size Nx2 or Nx3 or a symmetric distance matrix.
%   Euclidian distances are used in the coordinate case. M is an integer
%   in the range 1 to N. Default is M = 1.
%
%   METHOD
%   M nearest neighbour tours are generated from randomly selected starting
%   points. Each tour is improved by 2-opt heuristics (pairwise exchange of
%   edges) and the best result is selected.
%
%   EXAMPLES
%
%   X = rand(100,2);
%   [p,L] = tspsearch(X,100);
%   tspplot(p,X)
%
%   % Optimal tour length 1620
%   X = load('hex162.dat');
%   [p,L] = tspsearch(X,10);
%   tspplot(p,X)
%
%   % Optimal tour length 4860
%   X = load('hex486.dat');
%   [p,L] = tspsearch(X);
%   tspplot(p,X)

%   Author: Jonas Lundgren <splinefit@gmail.com> 2012

% Check first argument
[n,dim] = size(X);
if dim == 2 || dim == 3
    % X is a coordinate matrix, compute euclidian distances
    D = distmat(X); % i will give it to u 
elseif n == dim && min(X(:)) >= 0 && isequal(X,X')
    % X is a distance matrix
    D = X;
else
    mess = 'First argument must be Nx2, Nx3 or symmetric and nonnegative.';
    error('tspsearch:first',mess)
end

% Check second argument
if nargin < 2 || isempty(m)
    m = 1;
elseif ~isscalar(m) || m < 1 || m > n || fix(m) < m
    mess = 'M must be an integer in the range 1 to %d.';
    error('tspsearch:second',mess,n)
end

% Starting points for nearest neighbour tours
s=[n];
s=cat(2,s,randperm(n-1));
%s = randperm(n);

Lmin = inf;
for k = 1:m
    % Nearest neighbour tour
	p = greedy(s(k),D);
    % Improve tour by 2-opt heuristics
	[p,L] = exchange2(p,D);
    % Keep best tour
	if L < Lmin
        Lmin = L;
        pmin = p;
    end
    convergenceCurve(k) = Lmin;
end

% Output
p = double(pmin);
L = Lmin;


%--------------------------------------------------------------------------
function D = distmat(X)
%DISTMAT Compute euclidian distance matrix from coordinates

[n,dim] = size(X);
D = zeros(n);
for j = 1:n
    for k = 1:dim
        v = X(:,k) - X(j,k);
        D(:,j) = D(:,j) + v.*v;
    end
end
D = sqrt(D);
end
%--------------------------------------------------------------------------
function p = greedy(s,D)
%GREEDY Travel to nearest neighbour, starting with node s.

n = size(D,1);
p = zeros(1,n,'uint16');
p(1) = s;

for k = 2:n
    D(s,:) = inf;
    [junk,s] = min(D(:,s)); %#ok
    p(k) = s;
end
end
%--------------------------------------------------------------------------
function [p,L] = exchange2(p,D)
%EXCHANGE2 Improve tour p by 2-opt heuristics (pairwise exchange of edges).
%   The basic operation is to exchange the edge pair (ab,cd) with the pair
%   (ac,bd). The algoritm examines all possible edge pairs in the tour and
%   applies the best exchange. This procedure continues as long as the
%   tour length decreases. The resulting tour is called 2-optimal.

n = numel(p);
zmin = -1;

% Iterate until the tour is 2-optimal
while zmin < 0

    zmin = 0;
    i = 1;
    b = p(n);

    % Loop over all edge pairs (ab,cd)
    while i < n-2
        a = b;
        i = i+1;
        b = p(i);
        Dab = D(a,b);
        j = i+1;
        d = p(j);
        while j < n %
            c = d;
            j = j+1;
            d = p(j);
            % Tour length diff z
            % Note: a == d will occur and give z = 0
            z = (D(a,c) - D(c,d)) + D(b,d) - Dab;
            % Keep best exchange
            if z < zmin
                zmin = z;
                imin = i;
                jmin = j;
            end
        end
    end

    % Apply exchange
    if zmin < 0
        p(imin:jmin-1) = p(jmin-1:-1:imin);
    end

end

% Tour length
q = double(p);
ind = sub2ind([n,n],q,[q(2:n),q(1)]);
L = sum(D(ind))-(D(q(1),q(end)));

end
end
