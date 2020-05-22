function []=mp_pg(path)
Ld=3;
p_a=0;
T=20;
n_position=[-2.5 20];
s_position=[-2.5 0];
n=[-2.5 10];
d1=10;
start_point=path(1,:);
end_point=path(end,:);
f=path(end,2);
if end_point(2)==10
    p_a=pi;
    X=[end_point(1),end_point(2);n_position(1),n_position(2)];
    d2=pdist(X,'euclidean');
    T=d1;
    start_pos=[start_point(1) start_point(2) p_a];
    path_generator(T,path,start_pos,Ld);
else
    if f>0  %  its a cordinate in the northern node
        p_a=pi/2;
        X=[end_point(1),end_point(2);n_position(1),n_position(2)];
        d2=pdist(X,'euclidean');
        T=d1;
        start_pos=[start_point(1) start_point(2) p_a];
        y=path_generator(T,path(1:51,:),start_pos,Ld);
        p_a=0;
        T=d2;
        start_pos=[n_position(1) n_position(2) p_a];
        y=path_generator(T,path(51:end,:),start_pos,Ld);
        
    else   % its a cordinate in the southern node
        p_a=-pi/2;
        X=[end_point(1),end_point(2);s_position(1),s_position(2)];
        d2=pdist(X,'euclidean');
        T=d1;
        start_pos=[start_point(1) start_point(2) p_a];
        y=path_generator(T,path(1:51,:),start_pos,Ld);
        p_a=0;
        T=d2;
        start_pos=[s_position(1) s_position(2) p_a];
        y=path_generator(T,path(51:end,:),start_pos,Ld);
    end
end
