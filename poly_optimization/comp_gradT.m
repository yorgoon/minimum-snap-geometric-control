function T = comp_gradT(t)
    T = zeros(6);
    for i=1:length(T)
        for j=1:length(T)
            T(i,j) = t^(12-i-j);
        end
    end
end