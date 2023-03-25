function [L_start, L_end] = calc_offset_dist(x_list, y_list, end_x, end_y, end_theta)
    
    
    a = sin(end_theta * pi / 180);
    b = cos(end_theta  * pi / 180);
    if end_theta == 0   
        a = 1;
        b = 0;
    end
        
    L_end = (end_y - y_list(end)) / a;
    
    L_start = (end_x - x_list(end)) - L_end*b;

end
