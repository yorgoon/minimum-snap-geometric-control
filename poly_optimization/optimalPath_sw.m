function optimalPath = optimalPath_sw(map, path)


function c = check_col(map, pt1, pt2)
    for t=0:.01:1
        inter_pt = pt1*(1-t) + pt2*t;
        % check collision
        if collide(map, inter_pt)
            c = 1;
            return
        end
    end
    c = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

optimalPath = [];
optimalPath = [optimalPath; path(1,:)];

for m = 3:size(path,1)
    c = check_col(map, optimalPath(end,:), path(m,:));
    if c
        optimalPath = [optimalPath; path(m-1,:)];
    end
end
optimalPath = [optimalPath; path(end,:)];

end