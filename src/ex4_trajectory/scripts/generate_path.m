% ------------------------------------------------------------------------
% This function has been developed in the frame of the Control Systems
% Design class (a.y. 2018/2019).
%
% Title:    Generation of a generic path through splines
% Author:   Matteo Torraca, Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     Jul 2019
%
% See documentation below for a description of this function.
%
% ------------------------------------------------------------------------

function [x_lambda, lambda] = generate_path(ctrl_points, number_of_waypoints, draw)
    % generate_path Generates a multi-dimensional path through splining
    %   [x_lambda, lambda] = generate_path(CTRL_POINTS,
    %   NUMBER_OF_WAYPOINTS, DRAW) returns a joint-space or workspace path 
    %   of NUMBER_OF_WAYPOINTS samples through splines generated between 
    %   CTRL_POINTS. CTRL_POINTS is made up of as many rows as the number 
    %   of dimensions (e.g. 3 for three-dimensional paths, with no
    %   orientation) and as many columns as the number of control points. 
    %   If desired, the path can be visualized by passing DRAW=true. 
    %   In this case, the function generates a 2D plot if the control 
    %   points are two-dimensional or a 3D plot if the control points have 
    %   >=3 dimensions. The function returns the path X_LAMBDA (of the same 
    %   number of dimensions as the control points) and the underlying path 
    %   length LAMBDA.

    if draw ~= true && draw ~= false
        error('"draw" must be binary');
    end
    
    if number_of_waypoints <= 0
        error('"number_of_waypoints" must be greater than 0');
    end

    dimensions = length(ctrl_points(:,1));
    
    if dimensions <= 0
        error('"ctrl_points" is not a valid set of control points');
    end
    
    x_lambda    = NaN * ones(dimensions, number_of_waypoints);
    lambda      = NaN * ones(1,number_of_waypoints);

    t_start  = linspace(0,1,length(ctrl_points(1,:)));
    t_result = linspace(0,1,number_of_waypoints);

    for i=1:dimensions
        x_lambda(i,:) = spline(t_start, ctrl_points(i,:), t_result);
    end    
    
    lambda(1) = 0;
    
    for i=2:number_of_waypoints       
        lambda(i) = lambda(i-1) + norm(x_lambda(:,i-1)-x_lambda(:,i));
    end
    
    if draw
        figure;
        
        if dimensions == 1
            error('cannot visualize a one-dimensional path');
        end
        
        if dimensions == 2
            plot(x_lambda(1,:), x_lambda(2,:), 'linewidth', 3); 
        else
            plot3(x_lambda(1,:), x_lambda(2,:),x_lambda(3,:), 'linewidth', 3);
            zlabel('z');
        end
        
        xlabel('x'), ylabel('y')
        axis equal; grid on;
        
    end

end

