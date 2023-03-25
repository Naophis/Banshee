% ego
Large = 1; Orval = 2; Dia45 = 3; Dia135 = 4; Dia90 = 5; Dia180 = 6;
ego_v = 2050;
radius = 184;
sla.pow_n = 2;
target_angle = 180;
param_alpha = (2 * ego_v^2 / (radius^2 * (target_angle * pi / 180) / 2));
turn_mode = Dia180;
dt = 0.001/4;
is_dia_mode = false;

tmp_x = -45;
tmp_y = -45;

%start_offset_idx=6;
%end_offset_idx=6;

start_offset_idx = 0;
end_offset_idx = 0;

if is_dia_mode
    fprintf('start_point_s = %0.8f\r\n', sqrt(tmp_x^2 +tmp_y^2));
end

tmp_theta = 0;

if is_dia_mode
    tmp_theta = 45 * pi / 180;
end

alphaTemp = param_alpha;
sla.base_alpha = alphaTemp;
sla.counter = int32(0);

tmp_w = (0);
tmp_dist = (0);

tmp_x_list = zeros(100, 1);
tmp_y_list = zeros(100, 1);
tmp_w_list = zeros(100, 1);

tmp_x_list(1) = tmp_x;
tmp_y_list(1) = tmp_y;

i=1;
abs_ang=0;
while tmp_theta < (target_angle * pi / 180)
    tmp_alpha = alphaTemp;

    if (tmp_theta >= (target_angle * pi / 180) / 2)
        tmp_alpha = -(tmp_w^2)/(2*(abs(target_angle*pi/180)-tmp_theta));
    end

    tmp_w = tmp_w + tmp_alpha * dt;
    tmp_theta = tmp_theta + tmp_w * dt;
    abs_ang = abs_ang +abs(tmp_w * dt);
    if turn_mode == Dia180
        tmp_x = tmp_x + ego_v * cos(tmp_theta+45*pi/180) * dt;
        tmp_y = tmp_y + ego_v * sin(tmp_theta+45*pi/180) * dt;
    else 
        tmp_x = tmp_x + ego_v * cos(tmp_theta) * dt;
        tmp_y = tmp_y + ego_v * sin(tmp_theta) * dt;
    end
    tmp_x_list(i) = (tmp_x);
    tmp_y_list(i) = (tmp_y);
    tmp_w_list(i) = (tmp_w);
    i=i+1;
    if abs_ang+0.00001>= (target_angle * pi / 180)
       break; 
    end
end

figure(1);
plot(tmp_x_list, tmp_y_list, '-b');
G = 9.81;
fprintf('pos(x,y,rad,deg,max_G) = (%0.8f, %0.8f, %0.8f, %0.8f, %0.8fG)\r\n', tmp_x, tmp_y, tmp_theta, tmp_theta * 180 / pi, max(tmp_w_list)^2 * (radius / 1000) / G);

if turn_mode == Large || turn_mode == Orval
    fprintf('end_pos(x,y) = (%0.8f, %0.8f)\r\n', 180 - tmp_x, 180 - tmp_y);
elseif turn_mode == Dia45
    fprintf('end_pos(x,y) = (%0.8f, %0.8f)\r\n', 180 - tmp_x, 90 - tmp_y);
    s = sqrt((180 - tmp_x)^2 + (90 - tmp_y));
    fprintf('end_pos(s) = (%0.8f)\r\n', s);
elseif turn_mode == Dia135

    if ~is_dia_mode
        fprintf('end_pos(x,y) = (%0.8f, %0.8f)\r\n', 90 - tmp_x, 180 - tmp_y);
        fprintf('start_x = %0.8f\r\n', 270 - tmp_y - tmp_x);
        s = sqrt((90 - tmp_x)^2 + (180 - tmp_y));
        fprintf('end_pos(s) = (%0.8f)\r\n', s);
    else
        fprintf('end_pos(x,y) = (%0.8f, %0.8f)\r\n', 90 + tmp_x, 180 - tmp_y);
        % s = sqrt((90 - tmp_x)^2 + (180 - tmp_y));
        % fprintf('end_pos(s) = (%0.8f)\r\n', s);
    end

elseif turn_mode == Dia90

    if ~is_dia_mode
        fprintf('end_pos(x,y) = (%0.8f, %0.8f)\r\n', 90 - tmp_x, 180 - tmp_y);
        fprintf('start_x = %0.8f\r\n', 270 - tmp_y - tmp_x);
        s = sqrt((90 - tmp_x)^2 + (180 - tmp_y));
        fprintf('end_pos(s) = (%0.8f)\r\n', s);
    else
        fprintf('end_pos(x,y) = (%0.8f, %0.8f)\r\n', 90 + tmp_x, 180 - tmp_y);
        % s = sqrt((90 - tmp_x)^2 + (180 - tmp_y));
        % fprintf('end_pos(s) = (%0.8f)\r\n', s);
    end

end

if turn_mode == Dia180
    xlim([-270 180]);
    ylim([-90 320]);
    hold on;

    plot([-270 270], [0 0], 'k:');
    plot([-270 270], [180 180], 'k:');

    plot([0 0], [-90 270], 'k:');
    plot([180 180], [-90 270], 'k:');

    plot([-90 -90], [-180 270], 'k:');
    plot([90 90], [-180 270], 'k:');

    plot([-90 360], [-90 360], 'k:');
    plot([-90 270], [270 -90], 'k:');
    
    
    plot([-360 -90], [0 270], 'k:');
    

    plot([-180 6], [84 84], 'r:');
    plot([-180 6], [96 96], 'r:');

    plot([6 6], [84 96], 'r:');

    plot([174 174], [-90 270], 'r:');
    plot([186 186], [-90 270], 'r:');

    plot([-90 270], [264 264], 'r:');
    plot([-90 270], [276 276], 'r:');

    hold off;
elseif ~is_dia_mode
    xlim([-90 270]);
    ylim([-90 270]);
    hold on;

    plot([-90 270], [0 0], 'k:');
    plot([-90 270], [180 180], 'k:');

    plot([0 0], [-90 270], 'k:');
    plot([180 180], [-90 270], 'k:');

    plot([0 360], [-90 270], 'k:');
    plot([0 360], [270 -90], 'k:');

    plot([-90 270], [84 84], 'r:');
    plot([-90 270], [96 96], 'r:');

    plot([84 84], [-90 270], 'r:');
    plot([96 96], [-90 270], 'r:');

    hold off;
else
    xlim([-120 270]);
    ylim([-120 270]);

    hold on;

    plot([-90 270], [0 0], 'k:');
    plot([-90 270], [180 180], 'k:');

    plot([0 0], [-90 270], 'k:');
    plot([180 180], [-90 270], 'k:');

    plot([-90 -90], [-180 270], 'k:');
    plot([90 90], [-180 270], 'k:');

    plot([-90 360], [-90 360], 'k:');
    plot([-90 270], [270 -90], 'k:');

    plot([-180 6], [84 84], 'r:');
    plot([-180 6], [96 96], 'r:');

    plot([6 6], [84 96], 'r:');

    plot([174 174], [-90 270], 'r:');
    plot([186 186], [-90 270], 'r:');

    plot([-90 270], [264 264], 'r:');
    plot([-90 270], [276 276], 'r:');

    hold off;
end

figure(2);
plot(tmp_w_list)
xlim([-5 size(tmp_w_list, 1) * 1.1]);
ylim([-5 max(tmp_w_list) * 1.1]);
tmp_w_size = size(tmp_w_list, 1);

search_mode = 0;
start_offset_idx = 0;
end_offset_idx = 0;

for i = 1:1:tmp_w_size

    if tmp_w_list(i) == 0

        if search_mode == 0
            start_offset_idx = start_offset_idx + 1;
        else
            end_offset_idx = end_offset_idx + 1;
        end

    else
        search_mode = 1;
    end

end

fprintf("ignore_idx (start, end)= (%d %d)\r\n", start_offset_idx, end_offset_idx);
