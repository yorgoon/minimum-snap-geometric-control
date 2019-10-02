function video_gen(tsave, xsave, filename, target_fps, Fmat)

fv = stlread('quadrotor_model/quadrotor_base.stl');
fig = figure(1);
hold on
Q = FramePlot(tsave(1), xsave(1,:), Fmat(1,2:4), fv);
% axis([0 10.5 -1.5 20 0 7])
% axis([-3 2.5 -1.5 1.5 -1.5 1.5])
set(gcf,'Renderer','OpenGL')
v = VideoWriter(filename);
v.FrameRate = target_fps;
open(v)
for i=2:length(xsave(:,1))
    Q.UpdatePlot(tsave(i), xsave(i,:), Fmat(i,2:4))
    F = getframe(fig);
    writeVideo(v,F)
end
hold off
close(v);
end

