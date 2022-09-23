classdef StickModel
    properties
        fig
        axs
    end

    methods
        function self = StickModel()
            self.fig = figure;
            axis([-50, 250, -200, 200, 0, 300]);
            self.axs = self.fig.CurrentAxes;
            view(self.axs, [1 -2 0.5]);
            
            grid on
        end

        function close(self)
            arguments
                self StickModel
            end
            delete(self.fig);
        end

        function visualize(self, velocity, varargin)
            lastOrigin = [0; 0; 0; 1];
            for idx = 1:length(varargin)
                frame = varargin{idx};
                curOrigin = frame * [0; 0; 0; 1];
                self.drawAxes(frame);
                self.drawVec(lastOrigin, curOrigin);
                lastOrigin = curOrigin;
            end
            self.drawVec(curOrigin, curOrigin + [10 * velocity; 1], 'cyan');
        end
        
        function drawVec(self, from, to, color)
            arguments
                self StickModel
                from
                to
                color = 'black'
            end
            xyz = [from, to];
            line(self.axs, xyz(1, :), xyz(2, :), xyz(3, :), 'Color', color);
        end
        
        function drawAxes(self, tmat)
            origin = tmat * [0; 0; 0; 1];
            AXES_LEN = 30;
            basis = tmat * [AXES_LEN * eye(3); ones(1, 3)];
            self.drawVec(origin, basis(:, 1), 'red');
            self.drawVec(origin, basis(:, 2), 'green');
            self.drawVec(origin, basis(:, 3), 'blue');
        end

        function clear(self)
            delete(findobj(self.axs, "Type", "line"));
        end
    end
end