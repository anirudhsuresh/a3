%initilize 
% initilize the nodes and the cordinates 
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


XY = [X' Y'];

gamma_max = pi/4; gamma_min = -gamma_max; 
vmax = 1; v = vmax; vmin = 0;
tau_gamma = 0.001; tau_v= 0.001;gamma=0;

global dT;
global DT;
dT = 0.001; DT = 0.01; T = 60.0;

pos = [XY(1,1) XY(1,2) 0]; % initial position

count = 1;

path_tb_f=[-2.50000000000000,10;-2.50000000000000,9.80000000000000;-2.50000000000000,9.60000000000000;-2.50000000000000,9.40000000000000;-2.50000000000000,9.20000000000000;-2.50000000000000,9;-2.50000000000000,8.80000000000000;-2.50000000000000,8.60000000000000;-2.50000000000000,8.40000000000000;-2.50000000000000,8.20000000000000;-2.50000000000000,8;-2.50000000000000,7.80000000000000;-2.50000000000000,7.60000000000000;-2.50000000000000,7.40000000000000;-2.50000000000000,7.20000000000000;-2.50000000000000,7;-2.50000000000000,6.80000000000000;-2.50000000000000,6.60000000000000;-2.50000000000000,6.40000000000000;-2.50000000000000,6.20000000000000;-2.50000000000000,6;-2.50000000000000,5.80000000000000;-2.50000000000000,5.60000000000000;-2.50000000000000,5.40000000000000;-2.50000000000000,5.20000000000000;-2.50000000000000,5;-2.50000000000000,4.80000000000000;-2.50000000000000,4.60000000000000;-2.50000000000000,4.40000000000000;-2.50000000000000,4.20000000000000;-2.50000000000000,4;-2.50000000000000,3.80000000000000;-2.50000000000000,3.60000000000000;-2.50000000000000,3.40000000000000;-2.50000000000000,3.20000000000000;-2.50000000000000,3;-2.50000000000000,2.80000000000000;-2.50000000000000,2.60000000000000;-2.50000000000000,2.40000000000000;-2.50000000000000,2.20000000000000;-2.50000000000000,2;-2.50000000000000,1.80000000000000;-2.50000000000000,1.60000000000000;-2.50000000000000,1.40000000000000;-2.50000000000000,1.20000000000000;-2.50000000000000,1;-2.50000000000000,0.799999999999999;-2.50000000000000,0.600000000000000;-2.50000000000000,0.399999999999999;-2.50000000000000,0.199999999999999;-2.50000000000000,0;-2.50000000000000,0;-2.30000000000000,0;-2.10000000000000,0;-1.90000000000000,0;-1.70000000000000,0;-1.50000000000000,0;-1.30000000000000,0;-1.10000000000000,0;-0.900000000000000,0;-0.700000000000000,0;-0.500000000000000,0;-0.300000000000000,0;-0.0999999999999996,0;0.100000000000000,0;0.300000000000000,0;0.500000000000000,0;0.700000000000000,0;0.900000000000000,0;1.10000000000000,0];


q = [pos(1) pos(2) pos(3) gamma v];
u = [gamma v];
umin = [gamma_min vmin];
umax = [gamma_max vmax];

delta1=0; delta2=0; slip=0;

Qmin = [-Inf -Inf -Inf gamma_min -Inf];
Qmax = [Inf Inf Inf gamma_max Inf];

Ld = 2;

robot_tracker = zeros(int32(DT/T), 2);
error = zeros(1, int32(DT/T));
count = 1;


% h = waitbar(0, 'Calculating path');
complete = 0;
while complete == 0
    [gamma, cross_track_error] = purePursuitController(q, L, Ld, path);
    u = [gamma v];
    q_next = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v, delta1, delta2, slip);   
    
    robot_tracker(count,1:2) = q(1:2);
    error(1,count) = cross_track_error;
    count = count+1;
%     waitbar(t/T);
    q = q_next;
end
close(h);
% 
% build_tractor();
% figure(1); clf; hold on; axis equal;
% 
% z = [pos(1) pos(2) pos(3)];
% tr = create_transform_matrix(z);
% plot_tractor(tr, 'g');
% 
% plot(robot_tracker(1:end,1)', robot_tracker(1:end,2)', 'k')
% scatter(path(1:end,1)', path(1:end,2)', 'b');
% z = [q(1) q(2) q(3)];
% tr = create_transform_matrix(z);
% plot_tractor(tr, 'r');
% 
% figure(2); clf; hold on;
% plot(error);