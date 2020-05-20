

path_count = 1;
for i = 0:0.2:10
    path(path_count, 1:2) = [i, 0];
    path_count = path_count + 1;
end
for i = 0:0.2:5
    path(path_count, 1:2) = [10, i];
    path_count = path_count + 1;
end
for i = 10:0.2:20
    path(path_count, 1:2) = [i, 5];
    path_count = path_count + 1;
end

robot_tracker = zeros(int32(DT/T), 2);
error = zeros(1, int32(DT/T));
count = 5;

h = waitbar(0, 'Calculating path');
for t = 0:DT:T
    [gamma, cross_track_error] = purePursuitController(q, L, Ld, path);
    u = [gamma v];
    q_next = robot_bike_dyn(q, u, umin, umax, Qmin, Qmax, L, tau_gamma, tau_v, delta1, delta2, slip);   
    
    robot_tracker(count,1:2) = q(1:2);
    error(1,count) = cross_track_error;
    count = count+1;
    waitbar(t/T)
    q = q_next;
end
close(h)

build_tractor();
figure(1); clf; hold on; axis equal;

z = [pos(1) pos(2) pos(3)];
tr = create_transform_matrix(z);
plot_tractor(tr, 'g');

plot(robot_tracker(1:end,1)', robot_tracker(1:end,2)', 'k')
scatter(path(1:end,1)', path(1:end,2)', 'b');
z = [q(1) q(2) q(3)];
tr = create_transform_matrix(z);
plot_tractor(tr, 'r');

error = abs(error);
figure(2); clf; hold on;
histogram(error);

mn = mean(error);
mx = max(error);
pc = prctile(error, 95);
rm = rms(error);
fprintf("%d\n%d\n%d\n%d\n", mn, mx, pc, rm);