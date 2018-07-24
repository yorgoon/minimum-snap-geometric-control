classdef FramePlot < handle
    
    properties (SetAccess = public)
        t;
        state;
        k = 0;
        R;  % SO(3)
        d;  % translation
        g;  % SE(3)
        XData0; % x coords of meshes of the quadrotor model
        YData0; % y coords of meshes of the quadrotor model
        ZData0; % z coords of meshes of the quadrotor model
        Fmat;   % External force
        state_hist;       % position history
        state_des_hist;   % desired position history
        time_hist;        % time history
        stl_handle;       % STL file handle
    end
    properties (SetAccess = private)
        % Figure handles 
        h_3d;
        h_frame_x;
        h_frame_y;
        h_frame_z;
        h_pos_hist;
        h_patch_stl;
        h_ext_F;
    end
    
    methods
        function Q = FramePlot(t, state, Fmat, stl_handle)
            Q.state = state;
            Q.R = ROTZ(Q.state(9))*ROTX(Q.state(7))*ROTY(Q.state(8));
            Q.d = Q.state(1:3);
            Q.g = [Q.R Q.d';0 0 0 1];
            Q.stl_handle = stl_handle;
            Q.state_hist = zeros(6, 100);
            x_axis = Q.R(:,1);
            y_axis = Q.R(:,2);
            z_axis = Q.R(:,3);
            pt = Q.state(1:3);
            Q.t = t;
            Q.Fmat = Fmat;
            
            % initialization
            h_3d = gca;
            Q.h_3d = h_3d;
            hold(Q.h_3d, 'on')
            % Actual position history
            Q.h_pos_hist = plot3(Q.h_3d, Q.state(1), Q.state(2), Q.state(3), '.k', 'MarkerSize', 5);
            % Body frame
            Q.h_frame_x = plot3(Q.h_3d, [pt(1) x_axis(1)+pt(1)], ...
                                        [pt(2) x_axis(2)+pt(2)], ...
                                        [pt(3) x_axis(3)+pt(3)], 'r', 'LineWidth', 1.5);
            Q.h_frame_y = plot3(Q.h_3d, [pt(1) y_axis(1)+pt(1)], ...
                                        [pt(2) y_axis(2)+pt(2)], ...
                                        [pt(3) y_axis(3)+pt(3)], 'g', 'LineWidth', 1.5);
            Q.h_frame_z = plot3(Q.h_3d, [pt(1) z_axis(1)+pt(1)], ...
                                        [pt(2) z_axis(2)+pt(2)], ...
                                        [pt(3) z_axis(3)+pt(3)], 'b', 'LineWidth', 1.5);
            % external force
            Q.h_ext_F = plot3(Q.h_3d, [pt(1) Q.Fmat(1)/3+pt(1)], ...
                                      [pt(2) Q.Fmat(2)/3+pt(2)], ...
                                      [pt(3) Q.Fmat(3)/3+pt(3)], 'Color',[0.9290, 0.6940, 0.1250], 'LineWidth', 2.0);
            
            % Quadrotor model
            Q.h_patch_stl = patch(Q.stl_handle);
            % Model visual parameters
            Q.h_patch_stl.FaceColor = [0.25 0.25 0.25];
%             Q.h_patch_stl.EdgeColor = 'none';
            Q.h_patch_stl.FaceLighting = 'gouraud';
            Q.h_patch_stl.AmbientStrength = 0.15;
            axis('image')
            Q.XData0 = Q.h_patch_stl.XData;
            Q.YData0 = Q.h_patch_stl.YData;
            Q.ZData0 = Q.h_patch_stl.ZData;
            
            % transform the model to the starting point
            new_XData = zeros(size(Q.h_patch_stl.XData));
            new_YData = zeros(size(Q.h_patch_stl.YData));
            new_ZData = zeros(size(Q.h_patch_stl.ZData));
            
            for i=1:length(Q.h_patch_stl.XData(:))
                homo_coord = [Q.XData0(i) Q.YData0(i) Q.ZData0(i) 1]';
                transformed_homo_coord = Q.g*homo_coord;
                new_XData(i) = transformed_homo_coord(1);
                new_YData(i) = transformed_homo_coord(2);
                new_ZData(i) = transformed_homo_coord(3);
            end
            set(Q.h_patch_stl, 'XData', new_XData, 'YData', new_YData, 'ZData', new_ZData);
            hold(Q.h_3d, 'off')
            drawnow
        end
        % Update state
        function UpdateQuadState(Q, t, state, Fmat)
            Q.state = state;
            Q.R = ROTZ(Q.state(9))*ROTX(Q.state(7))*ROTY(Q.state(8));
            Q.d = Q.state(1:3);
            Q.g = [Q.R Q.d';0 0 0 1];
            Q.t = t;
            Q.Fmat = Fmat;

        end
        % Update history
        function UpdateQuadHist(Q)
            Q.k = Q.k + 1;
            Q.state_hist(:,Q.k) = Q.state(1:6);
        end      
      
        function UpdatePlot(Q, t, state, Fmat)
            Q.UpdateQuadState(t, state, Fmat);
            Q.UpdateQuadHist();
            
            x_axis = Q.R(:,1);
            y_axis = Q.R(:,2);
            z_axis = Q.R(:,3);
            pt = Q.state(1:3);
            
            set(Q.h_pos_hist, ...
            'XData', Q.state_hist(1,1:Q.k), ...
            'YData', Q.state_hist(2,1:Q.k), ...
            'ZData', Q.state_hist(3,1:Q.k));

            set(Q.h_frame_x, ...
            'XData', [pt(1) x_axis(1)+pt(1)], ...
            'YData', [pt(2) x_axis(2)+pt(2)], ...
            'ZData', [pt(3) x_axis(3)+pt(3)]);
            set(Q.h_frame_y, ...
            'XData', [pt(1) y_axis(1)+pt(1)], ...
            'YData', [pt(2) y_axis(2)+pt(2)], ...
            'ZData', [pt(3) y_axis(3)+pt(3)]);
            set(Q.h_frame_z, ...
            'XData', [pt(1) z_axis(1)+pt(1)], ...
            'YData', [pt(2) z_axis(2)+pt(2)], ...
            'ZData', [pt(3) z_axis(3)+pt(3)]);
            
            set(Q.h_ext_F, ...
            'XData', [pt(1) Q.Fmat(1)/3+pt(1)], ...
            'YData', [pt(2) Q.Fmat(2)/3+pt(2)], ...
            'ZData', [pt(3) Q.Fmat(3)/3+pt(3)]);        
            
            new_XData = zeros(size(Q.h_patch_stl.XData));
            new_YData = zeros(size(Q.h_patch_stl.YData));
            new_ZData = zeros(size(Q.h_patch_stl.ZData));
            
            for i=1:length(Q.h_patch_stl.XData(:))
                homo_coord = [Q.XData0(i) Q.YData0(i) Q.ZData0(i) 1]';
                transformed_homo_coord = Q.g*homo_coord;
                new_XData(i) = transformed_homo_coord(1);
                new_YData(i) = transformed_homo_coord(2);
                new_ZData(i) = transformed_homo_coord(3);
            end
            set(Q.h_patch_stl, 'XData', new_XData, 'YData', new_YData, 'ZData', new_ZData);
        end
    end
   
end