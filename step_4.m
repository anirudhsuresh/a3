route=[1,13,3,6,16,19,9,10,20,17,14,4,7,11,21,18,8,5,15,12,2,22];

global XY
s=route(2:21)
N=10;
R=3;
W=2.5;
pts=20;
j=0;
direction=0;
for i=1:19
    p=s(i);
    q=s(i+1);
        start_p=XY(p,:);
    %     end_p=XY(q,:);
    %     start_p
    %     end_p
    if abs(s(i)-s(i+1))==N      % its a straight line
        start_p=XY(i,:);
        end_p=XY(i+1,:);
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
        
        if dist_n>=2*R             % if f is greater than the min then pi turns
            path= pi_turn(R, dist_n,start_p, direction);
        else  %             else omega turn
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
D
 plot(D(:,1),D(:,2));pause(1)

