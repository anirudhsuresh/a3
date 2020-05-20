% function [XY,DMAT_old]=a3()
% assignment #3
% clc
% function [XY,DMAT_old]=a3()
% assignment #3
% clc
% clear all
% clf 
%initilizations
clear;figure(1); clf; hold on; axis equal;
W=2.5;                           % implementation width(m)
RL=20;                           % Length of field row (m)
N=10;                            % number of rows
X=zeros(1,22);Y=zeros(1,22);     % vector for x and y co-ordinates of the field

L = 3;
gamma_max = deg2rad(60);
R = (L/tan(gamma_max)) + W/2   ;  

X(1,2)=W/2;                      % X2 is W/2 away from the axis(0,0)
X(1,N+2)=W/2;
for i=2:N                        % since each co-ordinate is W away ||
    X(1,i+1)=X(1,i)+W;           % south x co-ordinates
    X(1,N+i+1)=X(1,N+i)+W;       % northern x co-ordinates
end

Y(1,2)=0;                        % X2 is W/2 away from the axis(0,0)
Y(1,N+2)=RL;

for i=2:N                        %since each co-ordinate is W away
    Y(1,i)=0;                    % south y co-ordinates
    Y(1,N+i+1)=RL;                 % northern y co-ordinates
end
% Y(1,21)=20;
X(1,1)=-W;                       % start node x' co-ordinate
X(1,2*N+2)=X(1,1);               % end node   x' co-ordinate

Y(1,1)=RL/2;                     % start node y' co-ordinate
Y(1,2*N+2)=Y(1,1);               % end node   y' co-ordinate

% to check the field visually
clf
plot(X,Y,'o')                  
grid on
x=1;
for i=1:22
    X_1(1,i)=X(1,i)+x;
    Y_1(1,i)=Y(1,i)+x;
end
for i=1:length(X)
  text(X_1(i),Y_1(i),num2str(i))
  hold on
end

DMAT_old=zeros(2*N+2,2*N+2);        %cost matrix %huge*ones(same matrix )
huge=10^10;                       % infinite value for the cost
small=-1;                        % zero value for the cost

%non turning costs
for i=2:2*N+1
 for j=N+2:2*N+1
    if (j-i)==N                 %both nodes are in same row 
        DMAT_old(i,j)=small;
        DMAT_old(j,i)=small;
    else                        % i and j are not of the same row  
        DMAT_old(i,j)=huge;
        DMAT_old(j,i)=huge;
    end 
 end 
end 

% pi and omega manuvers
% turning costs 
for i=2:N
    for j=i+1:N+1
        
        d=abs(i-j);              %calculate the distance 
        f=d*W;                   
        if f>=2*R             % if f is greater than the min then 
            cost=d*W+(pi-2)*R;             % pi turn
        else
            expo=(1-(((2*R+d*W)^2)/8*(R^2)));
            cost=3*pi*R-2*R*acos(expo);        % else omega turn 
        end
        DMAT_old(i,j)=cost;
        DMAT_old(j,i)=cost;
        DMAT_old(i+N,j+N)=cost;
        DMAT_old(j+N,i+N)=cost;
    end
end
% special cases 
% 1. costs from the start node and end node to other nodes
%    - we use manhattan distance

start_tnode=[X(1,1),Y(1,1)];      % start node co-ordinates 
% end_node=[X(1,22),Y(1,22)];      % end node co-ordinates 

for i=2:N+1
    
    m_dist_s=abs(start_tnode(1)-X(1,i))+abs(start_tnode(2)-Y(1,i));      % cost calculation for south nodes
    DMAT_old(1,i)=m_dist_s;                                                  % costs from start nodes to other nodes
    DMAT_old(i,1)=m_dist_s;                                                  % costs from  other nodes to start nodes
    DMAT_old(22,i)=m_dist_s;                                                 % costs from nodes to end node
    DMAT_old(i,22)=m_dist_s;                                                 % cost from other nodes to end node
    
    m_dist_n=abs(start_tnode(1)-X(1,N+i))+abs(start_tnode(2)-Y(1,N+i));  % cost calculation for northern nodes
    DMAT_old(1,N+i)=m_dist_n;                                                % costs from start nodes to other nodes
    DMAT_old(22,i+N)=m_dist_n;                                                 % costs from end node to other nodes
    DMAT_old(N+i,1)=m_dist_n;                                               % costs from other nodes to start nodes
    DMAT_old(N+i,22)=m_dist_n;                                                 % costs from nodes to end node
end 

% 2. Cost from start to end node and v/v
DMAT_old(1,22)=huge;
DMAT_old(22,1)=huge;

%3. from a one 'node' to the same 'node'
for i=1:2*N+2
    for i=1:2*N+2
        DMAT_old(i,i)=huge;
    end 
end

global XY 
XY = [X' Y'];
dmat=DMAT_old;
% global DMAT;
t = cputime;  
resultStruct = tspof_ga('xy', XY , 'DMAT', dmat, 'SHOWRESULT',false, 'SHOWWAITBAR', false, 'SHOWPROG', false); 
E = cputime-t; %print time required to compute it.   
route = [1 resultStruct.optRoute 2*N+2]; % extract node sequence   
display(route)
mini_dist=resultStruct.minDist;  %print computed minimum distance
display(mini_dist)

s=route;
for i=1:21
    co(i,1)=XY(i,1)';
    co(i,2)=XY(i,2)';
end

N=10;
R=3;
W=2.5;
pts=25;
j=0;
direction=0;
n=length(s);

rw = 0.5; L = 2.5;
gamma_max = pi/4; gamma_min = -gamma_max; 
vmax = 1; v = vmax; vmin = 0;
tau_gamma = 0.001; tau_v= 0.001; 
global dT;
global DT;
dT = 0.001; DT = 0.01; T = 25.0;

pos = [XY(1,1) XY(1,2) 0];
gamma = 0;

current_pos = [pos(1) pos(2) pos(3) gamma v];
u = [gamma v];
umin = [gamma_min vmin];
umax = [gamma_max vmax];

delta1=0; delta2=0; slip=0;

Qmin = [-Inf -Inf -Inf gamma_min -Inf];
Qmax = [Inf Inf Inf gamma_max Inf];

Ld = 3;
count = 1;

for i=1:n-1
    p=s(i);
    q=s(i+1);
    start_p=XY(p,:);
    end_p=XY(q,:);
    
    if i == 1 || i == n-1
        path = manhattan_path(start_p, end_p);
        
    elseif abs(p-q)==N      % its a straight line
%         start_p=XY(p,:);
%         end_p=XY(q,:);
        path=Line_way_p_gen(start_p,end_p,pts);
    else                        % its a headland manuver
        % to identify the direction of the current turn
        
        if p<=11 && q>=2
            if p>q
                direction='southwest';
            elseif p<q
                direction='southeast';
            end
        elseif p>=12 && q<=21
            if p<q
                direction='northeast';
            elseif p>q
                direction='northwest';
            end
        end
        
%         d=abs(s(i)-s(i+1));
%         dist_n=d*W;

        dist_n = abs(start_p(1) - end_p(1));
        
        if dist_n>=2*R      % if f is greater than the min then pi turns
            path= pi_turn(R, dist_n,start_p, direction);
        else                % else omega turn
            path= omega_turn(R, dist_n,start_p, direction);
             
        end
    end
    z=length(path);
    for k=1:z
        D(j+k,1)=path(k,1);
        D(j+k,2)=path(k,2);
    end
    j=j+k;
    path_exp=D;
    
    complete = 0;
    while complete == 0
        [gamma, complete] = purePursuitController(current_pos, L, Ld, path);
        u = [gamma v];
        current_pos_next = robot_bike_dyn(current_pos, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v, delta1, delta2, slip);   

        robot_tracker(count,1:2) = current_pos(1:2);
        count = count + 1;
        current_pos = current_pos_next;
    end
    
    scatter(path_exp(:,1),path_exp(:,2));
   
end

build_tractor();

robot_start = [pos(1) pos(2) pos(3)];
tr = create_transform_matrix(robot_start);
plot_tractor(tr, 'g');

plot(robot_tracker(1:end,1)', robot_tracker(1:end,2)', 'k')
robot_end = [current_pos(1) current_pos(2) current_pos(3)];
tr = create_transform_matrix(robot_end);
plot_tractor(tr, 'r');

