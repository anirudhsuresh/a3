route=[1,13,3,6,16,19,9,10,20,17,14,4,7,11,21,18,8,5,15,12,2,22];

global XY
XY
% to determine from the route sequence if the nodes are of the same line or
% are turning between nodes 
% assume the same r_min =2 and W=2.5

s=route(2:21);
N=10;
R=2;
% array=zeros();
W=2.5;
L = 3;
gamma_max = deg2rad(60);
R = (L/tan(gamma_max)) + W/2
% step one is determine if the point in the same row or not
% for i=s(1:end)
%         if abs(s(i)-s(i+1 ))==N
%             array=[s(i),s(i+1)];
%             %call in a function to generate way points on a line
%         else 
%             d=abs(i-j);
%             f=d*W;
%             if f>=2*R             % if f is greater than the min then
%                 % pi turn generate a way-points for the pi turn
%             else
%                  % else omega turn and generate way-points for an omega
%                  % turn
%             end
%         end 
% end 
pts=3;
% D=zeros(:,2);
j=0;
for i=1:19
    if abs(s(i)-s(i+1))==N
                XY(i,:);
                XY(i+1,:);
                startp=XY(i,:);
                endp=XY(i+1,:);
                [xx,yy]=fillline(1,10,10);
                n=length(xx);
    else
        d=abs(i-(i+1));
        f=d*W;
        if f>=2*R             % if f is greater than the min then
            %             pi turn generate a way-points for the pi turn
            
        else
            %             else omega turn and generate way-points for an omega
            
        end
    end

    n=length(xx);
    for k=1:n
        D(j+k,1)=xx(k)';
        D(j+k,2)=yy(k)';
    end
    j=j+k;
end
D
% end
% take starting, ending point & number of points in between
% make line to connect between 2 coordinates


