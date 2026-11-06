function [xk1,yk1,xk,yk,last] = WP_selector(x_pos,y_pos)
last = false;

persistent waypoints
if isempty(waypoints)
    waypoints = load('WP.mat').WP;
end

persistent current_WP_index 
if isempty(current_WP_index)
    current_WP_index = 1;
end

next_WP_index = current_WP_index +1;

current_waypoint_coordinates = waypoints(:,current_WP_index);
next_waypoint_coordinates = waypoints(:,next_WP_index);

dx = x_pos-next_waypoint_coordinates(1);
dy = y_pos-next_waypoint_coordinates(2);
R = (dx)^2 + (dy)^2;

L_pp = 161;
look_ahead = 1.5*L_pp;
R_acceptance = look_ahead;

if R <= R_acceptance^2
    current_WP_index = next_WP_index;
    if current_WP_index >= size(waypoints,2)
        last = true;
        current_WP_index = size(waypoints,2);
    end
end

xk = waypoints(1,current_WP_index);
yk = waypoints(2,current_WP_index);
xk1 = waypoints(1,min(current_WP_index + 1,size(waypoints,2)));
yk1 = waypoints(2,min(current_WP_index + 1,size(waypoints,2)));

end