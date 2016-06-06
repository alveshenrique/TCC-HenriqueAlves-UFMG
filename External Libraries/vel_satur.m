function [vel_right, vel_left] = vel_satur(right, left)
vel_max_pos = 0.4;
vel_max_neg = -0.4;
vel_min_pos = 0.025;
vel_min_neg = -0.025;
if right>0
    if right>=vel_min_pos && right<=vel_max_pos
        vel_right = right;
    else
        if right<vel_min_pos
            vel_right = 0;
        else
            vel_right = vel_max_pos;
        end
    end
else
    if right<=vel_min_neg && right>=vel_max_neg
        vel_right = right;
    else
        if right>vel_min_neg
            vel_right = 0;
        else
            vel_right = vel_max_neg;
        end
    end
end  

if left>0
    if left>=vel_min_pos && left<=vel_max_pos
        vel_left = left;
    else
        if left<vel_min_pos
            vel_left = 0;
        else
            vel_left = vel_max_pos;
        end
    end
else
    if left<=vel_min_neg && left>=vel_max_neg
        vel_left = left;
    else
        if left>vel_min_neg
            vel_left = 0;
        else
            vel_left = vel_max_neg;
        end
    end
end
end