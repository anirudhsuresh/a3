% clear; figure(1); clf; hold on; axis equal;
% 
% start = [5 -5];
% finish = [1 1];

function path = manhattan_path(start, finish)
    path_count = 1;

    path(1,1:2) = start;
    if start(1) < finish(1) && start(2) < finish(2)
        for i=start(2):0.2:finish(2)
           path(path_count,1:2) = [start(1),i];
           path_count = path_count+1;
        end
        for i=start(1):0.2:finish(1)
           path(path_count,1:2) = [i,path(end,2)];
           path_count = path_count+1;
        end
    end

    if start(1) > finish(1) && start(2) > finish(2)
        for i=finish(2):0.2:start(2)
           path(path_count,1:2) = [finish(1),i];
           path_count = path_count+1;
        end
        for i=finish(1):0.2:start(1)
           path(path_count,1:2) = [i,path(end,2)];
           path_count = path_count+1;
        end
        path = flipud(path);
    end

    if start(1) < finish(1) && start(2) > finish(2)
        for i=start(2):-0.2:finish(2)
           path(path_count,1:2) = [start(1),i];
           path_count = path_count+1;
        end
        for i=start(1):0.2:finish(1)
           path(path_count,1:2) = [i,path(end,2)];
           path_count = path_count+1;
        end
    end

    if start(1) > finish(1) && start(2) < finish(2)
        temp = start;
        start = finish;
        finish = temp;
        for i=start(2):-0.2:finish(2)
           path(path_count,1:2) = [start(1),i];
           path_count = path_count+1;
        end
        for i=start(1):0.2:finish(1)
           path(path_count,1:2) = [i,path(end,2)];
           path_count = path_count+1;
        end
        path = flipud(path);
    end

%     path
%     scatter(path(1:end,1)', path(1:end,2)', 'b');
end