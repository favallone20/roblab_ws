% ------------------------------------------------------------------------
% This function has been developed as part of the research activities of
% the Automatic Control Group at UNISA
%
% Title:    Export of workspace paths to be loaded in ROS
% Author:   Enrico Ferrentino
% Org.:     UNISA - Automatic Control Group
% Date:     Nov 2019
%
% See documentation below for a description of this function.
%
% ------------------------------------------------------------------------

function [] = export_ros_workspace_path(filename, time, path)
% export_ros_workspace_path Exports a path to file in .traj format
%   [] = export_ros_workspace_path(FILENAME, TIME, PATH) exports the PATH
%   to FILENAME, as well as the TIME vector of timestamps associated with
%   the samples in PATH. Every column in path corresponds to a waypoint and
%   must have 6 rows corresponding to x, y, z, roll, pitch and yaw
%   coordinates.

    number_of_waypoints = length(time);
    
    fileID = fopen(filename,'w');
    
    fwrite(fileID, number_of_waypoints, 'uint32');
    
    for i=1:number_of_waypoints
       
        fwrite(fileID, time(i), 'double');
        fwrite(fileID, path(1,i), 'double');
        fwrite(fileID, path(2,i), 'double');
        fwrite(fileID, path(3,i), 'double');
        
        quaternion = angle2quat(path(6,i), path(5,i), path(4,i), 'zyx');
        
        fwrite(fileID, quaternion(2), 'double');
        fwrite(fileID, quaternion(3), 'double');
        fwrite(fileID, quaternion(4), 'double');
        fwrite(fileID, quaternion(1), 'double');
        
    end

end

