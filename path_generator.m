function [final_pose, error]=path_generator(T,path,start_pos,Ld)
L = 3;
gamma_max = pi/4; gamma_min = -gamma_max; 
vmax = 1; v = vmax; vmin = 0;
tau_gamma = 0.001; tau_v= 0.001;
% tau_gamma = 0.30; tau_v = 0.5; 
global dT;
global DT;
dT = 0.001; DT = 0.01; 
gamma = 0;
% start_pos=start_pos;
q = [start_pos(1) start_pos(2) start_pos(3) gamma v];
u = [gamma v];
umin = [gamma_min vmin];
umax = [gamma_max vmax];

delta1=0; delta2=0; slip=0;

Qmin = [-Inf -Inf -Inf gamma_min -Inf];
Qmax = [Inf Inf Inf gamma_max Inf];

% Ld = 2;

robot_tracker = zeros(int32(DT/T), 2);
error = zeros(1, int32(DT/T));
count = 1;


h = waitbar(0, 'Calculating path');
for t = 0:DT:T
    [gamma, cross_track_error] = purePursuitController(q, L, Ld, path);
    u = [gamma v];
    q_next = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v, delta1, delta2, slip);   
    
    robot_tracker(count,1:2) = q(1:2);
    error(1,count) = cross_track_error;
    count = count+1;
    
    waitbar(t/T);
    q = q_next;
end
final_pose=[q(1) q(2)];
close(h);
build_tractor();
figure(1); hold on; axis equal;

% z = [start_pos(1) start_pos(2) start_pos(3)];
% tr = create_transform_matrix(z);hgb
% plot_tractor(tr, 'g');

plot(robot_tracker(1:end,1)', robot_tracker(1:end,2)', 'k')
% scatter(path(1:end,1)', path(1:end,2)', 'b');

% z = [q(1) q(2) q(3)];
% tr = create_transform_matrix(z);
% plot_tractor(tr, 'r');

figure(2); clf; hold on;
plot(error);