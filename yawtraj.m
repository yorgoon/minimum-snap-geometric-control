% Check trajectory
ttt = [];
ttt = [ttt, linspace(0,tau_vec(1),50)];
for i=2:length(tau_vec)
    ttt = [ttt, sum(tau_vec(1:i-1)) + linspace(0,tau_vec(i),50)];
end

XX = [];
for i=1:length(tau_vec) % # of segments
    X = polyval(P_yaw(6*i:-1:6*(i-1)+1),linspace(0,tau_vec(i),50));
    plot(ttt(50*(i-1)+1:50*i),X)
    grid on
    hold on
end
hold off