function [final_position]=pi_turn_pg(path,d3,start_position)
global error;
global error_count;
T=d3;
Ld=3;
f=path(1,2);
if start_position==0
    start_point=path(1,:);
else 
    start_point=start_position;
end
    if f>12
        p_a=pi/2;
    else
        p_a=-pi/2;
    end
start_pos=[start_point(1) start_point(2) p_a];
[final_position, e]=path_generator(T,path,start_pos,Ld);
error_count = error_count + 1;
error(1,error_count) = e;


    