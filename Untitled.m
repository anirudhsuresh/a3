% % % route=[1,13,3,6,16,19,9,10,20,17,14,4,7,11,21,18,8,5,15,12,2,22];
% % % % south :2 to 11 
% % % % north :12 to 
% % % % east :if the next node increases 
% % % % west : if the next node decrease 
% % % s=route(2:21);
% % Line_way_p_gen
% % start_p=[0 0];
% % end_p=[10 10];
% % pts=100;
% % 
% % patJJ=fillline(start_p,end_p,pts);
% % function path=fillline(start_p,end_p,pts)
% %         m=(end_p(2)-start_p(2))/(end_p(1)-start_p(1)); %gradient 
% %         if m==Inf %vertical line
% %             xx(1:pts)=start_p(1);
% %             yy(1:pts)=linspace(start_p(2),end_p(2),pts);
% %         elseif m==0 %horizontal line
% %             xx(1:pts)=linspace(start_p(1),end_p(1),pts);
% %             yy(1:pts)=start_p(2);
% %         else %if (endp(1)-startp(1))~=0
% %             xx=linspace(start_p(1),end_p(1),pts);
% %             yy=m*(xx-start_p(1))+start_p(2);
% %         end
% %         path=[xx',yy'];
% % end
% route=[1,13,3,6,16,19,9,10,20,17,14,4,7,11,21,18,8,5,15,12,2,22];
% s=route(2:21);
% p=5
% q=6
% direction_p;
%            if s(p)<=11 && s(q)>=2
%               if s(p)>s(q)
%                   direction_p='southwest';
%               elseif s(p)<s(q)
%                   direction_p='southeast';
%               end
%           elseif s(p)>=12 && s(q)<=21
%               if s(p)>s(q)
%                   direction_p='northeast';
%               elseif s(p)<s(q)
%                   direction_p='northwest';
%               end
%           end
route=[1,13,3,6,16,19,9,10,20,17,14,4,7,11,21,18,8,5,15,12,2,22];
s=route(2:21);
N=10
% % i=14
% % % for i=1:19
% %     p=s(i);
% %     q=s(i+1);
% % %     start_p=XY(p,:);
% % %     end_p=XY(q,:);
% %     if abs(s(i)-s(i+1))==N      % its a straight line
% %         
% %     else                        % its a headland manuver
% %         % to identify the direction of the current turn
% %         s(p)
% %         s(q)
% %         if s(p)<=11 && s(q)>=2
% %             if s(p)>s(q)
% %                 direction='southwest';
% %             elseif s(p)<s(q)
% %                 direction='southeast';
% %             end
% %         elseif s(p)>=12 && s(q)<=21
% %             if s(p)>s(q)
% %                 direction='northeast';
% %             elseif s(p)<s(q)
% %                 direction='northwest';
% %             end
% %         end
% %     end
% % % end 
D_GGG=0
for i=1:19
    if abs(s(i)-s(i+1))==N
                XY(i,:);
                XY(i+1,:);
                startp=XY(i,:);
                endp=XY(i+1,:);
                path_ccc=Line_way_p_gen(startp,endp,10);
%                 n=length(xx);
%     else
%         d=abs(i-(i+1));
%         f=d*W;
%         if f>=2*R             % if f is greater than the min then
%             %             pi turn generate a way-points for the pi turn
%             
%         else
%             %             else omega turn and generate way-points for an omega
%             
%         end
%     end
% 
%     n=length(xx);
%     for k=1:n
%         D_GGG(j+k,1)=xx(k)';
%         D_GGG(j+k,2)=yy(k)';
%     end
%     j=j+k;
    end 
end


