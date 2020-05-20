function q1 = robot_bike_dyn(q, u, umin, umax,Qmin, Qmax, L, tau_gamma, tau_v, delta1, delta2, slip)
global dT;
global DT;
% model of a vehicle with closed loop steering and velocity control
% the combined effect of steering/vehicle inertia and control is
% a first order system for steering (tau_phi) and velocity (tau_v)
% state is
% q(1) -> x
% q(2) -> y
% q(3) -> theta (orientation in world frame)
% q(4) -> phi (steering angle)
% q(5) -> linear velocity

% inputs are:
% u(1) -> desired steering angle
% u(2) -> desired linear velocity

%impose control constraints, i.e., restrict input to lie between Umin and Umax
for i=1 : length (u)
    if u(i) > umax(i)
        u(i) = umax(i);
    else
        if u(i) < umin(i)
            u(i) = umin(i);
        end
    end
    
end % for
qk=q;

% u is control input, u=[V phi], desired linear speed and steering angle
V_d = u(2); % m/s, desired velocity
gamma_d = u(1); % desired steering angle in rads

for t=0:dT:DT-dT    %integrate model assuming constant inputs during DT
    
    Vl = (1-slip)*qk(5); % vehicle long. velocity state (at back wheel)
    Vy = Vl*tan(delta2); % vehicle lat. velocity state (at back wheel)
    
    q1(1) = qk(1)+ dT * (cos(qk(3)) * Vl - sin(qk(3)) * Vy); % xdot
    q1(2) = qk(2)+ dT * (sin(qk(3)) * Vl + cos(qk(3)) * Vy); %ydot
    q1(3) = qk(3) + dT * (qk(5) * tan(qk(4)+delta1) - Vy)/L;%theta dot
    q1(4) = qk(4) + dT * (-qk(4)+gamma_d)/tau_gamma; % gamma dot
    q1(5) = qk(5)+ dT * (-qk(5)+V_d)/tau_v; % Vldot
    
    
    %impose state constraints
    for i=1 : length (q1)
        if q1(i) > Qmax(i)
            q1(i) = Qmax(i);
        else
            if q1(i) < Qmin(i)
                q1(i) = Qmin(i);
            end
        end
        
    end % for
    qk = q1;
end

