function [x, y, w, theta, idx] = detect_trj_point(ego, tgt, mode)
    dist = ego.dist;
    trj_shape = tgt.trajectory_point;
    trj_shape_size = tgt.trajectory_point_size;
    x = single(0);
    y = single(0);
    w = single(0);
    theta = single(0);
    idx = int32(1);

    if dist < 0
        return;
    end

    max_dist = trj_shape(trj_shape_size).dist;

    if dist >= max_dist
        idx = trj_shape_size;
        % x = trj_shape(idx).x;
        y = trj_shape(idx).y;
        w = trj_shape(idx).w;
        theta = trj_shape(idx).theta;
        return;
    end

    tmp_i_start = int32((dist - 1) * 4);
    tmp_i_start = max(tmp_i_start, 1);
    tmp_i_start = min(tmp_i_start, trj_shape_size);

    tgt_idx = int32(tmp_i_start);

    for i = tmp_i_start:1:trj_shape_size

        if trj_shape(i).dist >= dist
            break;
        end

        tgt_idx = int32(i);
    end

    idx_1 = tgt_idx - int32(1);
    idx_2 = tgt_idx;

    if idx_1 == 0
        return;
    end

    point_1 = trj_shape(idx_1);
    point_2 = trj_shape(idx_2);

    idx = tgt_idx;
    x = lerp(point_1.dist, point_1.x, point_2.dist, point_2.x, dist);
    y = lerp(point_1.dist, point_1.y, point_2.dist, point_2.y, dist);
    w = lerp(point_1.dist, point_1.w, point_2.dist, point_2.w, dist);
    theta = lerp(point_1.dist, point_1.theta, point_2.dist, point_2.theta, dist);

end

function res = lerp(x1, y1, x2, y2, x)
    res = y1 + (y2 - y1) * (x - x1) / (x2 - x1);
end
