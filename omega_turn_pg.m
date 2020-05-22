function [final_position]=omega_turn_pg(path,start_position)
T=18.9;
Ld=0.78;
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
final_position=path_generator(T,path,start_pos,Ld);