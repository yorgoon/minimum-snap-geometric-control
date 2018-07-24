function plot_frame(pt,R)
framesize = 1;
x_axis = R(:,1)*framesize;
y_axis = R(:,2)*framesize;
z_axis = R(:,3)*framesize;
plot3([pt(1) x_axis(1)+pt(1)],[pt(2) x_axis(2)+pt(2)], [pt(3) x_axis(3)+pt(3)], 'b')
%hold on
plot3([pt(1) y_axis(1)+pt(1)],[pt(2) y_axis(2)+pt(2)], [pt(3) y_axis(3)+pt(3)], 'g')
plot3([pt(1) z_axis(1)+pt(1)],[pt(2) z_axis(2)+pt(2)], [pt(3) z_axis(3)+pt(3)], 'r')
%hold off