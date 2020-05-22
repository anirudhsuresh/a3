function []=do_path_gen_plot(route,XY,N,R)
s=route;

pts=25;
j=0;
direction=0;
n=length(s);
final_position=0;
for i=1:n-1
    p=s(i);
    e=s(i+1);
    start_p=XY(p,:);
    end_p=XY(e,:);
    
    if i == 1 || i == n-1
        path = manhattan_path(start_p, end_p);
        mp_pg(path); %generate manhattan path's robot trace 
        
    elseif abs(p-e)==N      % its a straight line
        path=Line_way_p_gen(start_p,end_p,pts);
        if p>e
            % north to south node 
            path=flip(path);
        else 
            path=path;
        end 
      final_position=st_line_pg(path,start_position);
    else                        % its a headland manuver
        % to identify the direction of the current turn
        
        if p<=11 && e>=2
            if p>e
                direction='southwest';
            elseif p<e
                direction='southeast';
            end
        elseif p>=12 && e<=21
            if p<e
                direction='northeast';
            elseif p>e
                direction='northwest';
            end
        end
        dist_n = abs(start_p(1) - end_p(1));
        
        if dist_n>=2*R      % if f is greater than the min then pi turns
            path= pi_turn(R, dist_n,start_p, direction);
            d2=(pi-2)*R;
            d3=dist_n+d2;
            final_position=pi_turn_pg(path,d3,start_position);
        else                % else omega turn
            path= omega_turn(R, dist_n,start_p, direction);
            final_position=omega_turn_pg(path,start_position);
        end
    end
    z=length(path);
    for k=1:z
        D(j+k,1)=path(k,1);
        D(j+k,2)=path(k,2);
    end
    j=j+k;
    path_exp=D;
    scatter(path_exp(:,1),path_exp(:,2));
    start_position=final_position;
end

