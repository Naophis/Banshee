[x, y, w, theta, idx] = detect_trj_point(trajectory_shape_list, trajectory_shape_list_size, 1);
fprintf('%f, %f, %f, %f, %d\r\n', x, y, w, theta*180/pi, idx);
