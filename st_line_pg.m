function [final_position]=st_line_pg(path,start_position)
T=20;
Ld=3;
if start_position==0
    start_point=path(1,:);
else 
    start_point=start_position;
end 
pnt_c=path(1,:);
f=pnt_c(2);
    if f==20 || f>10
        p_a=-pi/2;
    else
        p_a=pi/2;
    end
start_pos=[start_point(1) start_point(2) p_a];
final_position=path_generator(T,path,start_pos,Ld);

