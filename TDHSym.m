classdef TDHSym
    methods(Static)
        %% Denavit-Hartenburg Transformation
        function T = tdh(theta, d, a, alpha)
            T = TDHSym.trotz(theta) * TDHSym.transmat([a; 0; d]) * TDHSym.trotx(alpha);
        end
        function T = tdh2(params)
            T = TDHSym.tdh(params(1), params(2), params(3), params(4));
        end
        
        %% Rotations
        function R = rot2d(theta)
            % check for special cases to make symbolic computations more
            % stable
            if theta == 0
                R = eye(2);
            elseif abs(theta) == pi / 2
                R = [0 -1;
                     1  0] * sign(theta);
            elseif abs(theta) == pi
                R = [-1 0;
                      0 -1];
            else
                R = [cos(theta) -sin(theta);
                     sin(theta)  cos(theta)];
            end
        end
        
        function T = trotz(theta)
            T = sym(eye(4));
            T(1:2, 1:2) = TDHSym.rot2d(theta);
        end
        
        function T = troty(theta)
            T = sym(eye(4));
            T(1:2:3, 1:2:3) = TDHSym.rot2d(theta);
        end
        
        function T = trotx(theta)
            T = sym(eye(4));
            T(2:3, 2:3) = TDHSym.rot2d(theta);
        end
        
        %% Translation
        function T = transmat(offset)
            T = sym(eye(4));
            T(1:3, 4) = offset;
        end
    end
end