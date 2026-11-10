function [pi_h, y_ep] = crossTrackError(xk1,yk1,xk,yk, x_pos,y_pos)

delta_y = yk1-yk;
delta_x = xk1-xk;

alpha_k = atan2(delta_y,delta_x);
e = -(x_pos - xk)*sin(alpha_k) + (y_pos -yk)*cos(alpha_k);

pi_h = alpha_k;
y_ep = e;

end