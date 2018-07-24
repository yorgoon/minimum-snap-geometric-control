function bf = const_bf(PATH)

% Path (x,y,z) by nodes
m = size(PATH,1);

bf = zeros(10*(m-1),size(PATH,2));
bf(1,:) = PATH(1,:);
bf(end-4,:) = PATH(end,:);
idx = 1;
for i=1:m-2
    bf(5*(idx)+1,:) = PATH(i+1,:);
    idx = idx+1;
    bf(5*(idx)+1,:) = PATH(i+1,:);
    idx = idx+1;
end
idx = [];
for i=1:m-2
    idx = [idx, 10*i+2:10*i+5];
end
bf(idx,:) = [];
