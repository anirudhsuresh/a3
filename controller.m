% global DMAT;
% global XY 
[XY,DMAT]=a3();
% t = cputime;  
resultStruct = tspof_ga('xy', XY , 'DMAT', DMAT, 'SHOWRESULT',false, 'SHOWWAITBAR', false, 'SHOWPROG', false); 
% E = cputime-t; %print time required to compute it.   
% route = [1 resultStruct.optRoute 2*N+2]; % extract node sequence   
% display(route)
% mini_dist=resultStruct.minDist;  %print computed minimum distance
% display(mini_dist)
route=[1,13,3,6,16,19,9,10,20,17,14,4,7,11,21,18,8,5,15,12,2,22];


% to determine from the route sequence if the nodes are of the same line or
% are turning between nodes 
% assume the same r_min =2 and W=2.5

s=route(2:21);
