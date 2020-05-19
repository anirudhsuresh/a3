route=[1,13,3,6,16,19,9,10,20,17,14,4,7,11,21,18,8,5,15,12,2,22];

global XY
s=route(2:21);
for i=1:21
    co(i,1)=XY(i,1)';
    co(i,2)=XY(i,2)';
end
N=10;
R=3;
W=2.5;
pts=5;
j=0;
direction=0;
n=length(s);
for i=1:n-1
    p=s(i);
    q=s(i+1);
    start_p=XY(p,:);
    end_p=XY(q,:);
    
    if abs(p-q)==N      % its a straight line
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
            if p>q
                direction='northeast';
            elseif p<q
                direction='northwest';
            end
        end
        
        d=abs(s(i)-s(i+1));
        dist_n=d*W;
        
        if dist_n>=2*R      % if f is greater than the min then pi turns
            path= pi_turn(R, dist_n,start_p, direction);
        else                % else omega turn
            path= omgea_turn(R, dist_n,start_p, direction);
        end
    end
    n=length(path);
    for k=1:n
        D(j+k,1)=path(k,1);
        D(j+k,2)=path(k,2);
    end
    j=j+k;
 
end
path_exp=D;
% path_m=manhattan_path(XY(s(1),:),XY(s(2),:));
% start_path=path_m;
% path_m=manhattan_path(XY(s(20),s(21)));
% end_path=path_m;
 plot(path_exp(:,1),path_exp(:,2))

