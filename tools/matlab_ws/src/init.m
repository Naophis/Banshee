clear;

home = pwd;

sim_time = 25;
script_file = mfilename('fullpath');
[filepath, name, ext] = fileparts(script_file);
cd(filepath);
Normal = 0; Large = 1; Orval = 2; Dia45 = 3; Dia135 = 4; Dia90 = 5; Dia180 = 6; Dia45_2 = 7; Dia135_2 = 8;
Simulink.importExternalCTypes('../include/bus.h');

dt = 0.001;

 
test_mode = 0; % straight

test_mode = 1; % slalom
 %test_mode = 2; % pivot
%  test_mode = 3; % back_straight
% test_mode = 4; % slalom2
% test_mode = 5; % back

%default
sla.base_alpha = 0;
sla.base_time = 0;
sla.limit_time_count = 0;
sla.pow_n = 0;
sla.state = 0;
sla.counter = int32(0);

shape_list_max_size = int32(15);
trj_format = struct('x', single(0), 'y', single(0), 'w', single(0), 'theta', single(0), 'dist', single(0));
trajectory_point = struct('x', single(0), 'y', single(0), 'w', single(0), 'theta', single(0), 'dist', single(0));
trajectory_shape_list = repmat(trajectory_point, shape_list_max_size, 1);
trajectory_shape_list_size = int32(0);

k_x = single(0.0);
k_y = single(0.05);
k_theta = single(0.0);

accl_param_limit = single(4500);
accl_param_n = single(16);
slip_gain = 400;

k1 = 0.5;
k2 = 135;

mass = single(0.015);
lm=single(0.000065);
ke=single(0.00125);
km=single(0.0005);
resist=single(1.7);
tread=single(38);
tire=single(13);
gear_ratio=single(37/9);


% mass = single(0.11);
% lm=single(0.00025);
% km=single(0.00352);
% resist=single(2.93);
% tread=single(74);
% ke=single(0.000368613538);
% tire=single(23);
% gear_ratio=single(67/21);


if test_mode == 0
    % tgt
    v_max = 3800;
    end_v = 200;
    accl = 12000;
    decel = -12000;

    w_max = 0;
    end_w = 0;
    param_alpha = 0;
    tgt_dist = 90*32;
    tgt_angle = 0;

    % ego
    ego_v = 500;
    ego_accl = 0;
    ego_w = 0;
    ego_alpha = 0;
    ego_dist = 0;
    ego_state = int8(0);

    PhyBase.accl_limitter_x = [0, 2500, 3500, 4500, 6500];
    PhyBase.accl_limitter_gain = [1, 1, 0.8, 0.6, 0.2];

    radius = 120;
    alphaTemp = (ego_v / radius);
    sla.base_alpha = alphaTemp;
    sla.base_time = 0.08233642578125;
    sla.limit_time_count = sla.base_time * 2 / dt;
    sla.pow_n = 4;
    sla.state = 0;
    sla.counter = int32(1);

elseif test_mode == 1
    % tgt
    v_max = 950;
    end_v = v_max;
    accl = 28000;
    decel = -20000;
    w_max = 0;
    end_w = 0;
    param_alpha = 0;
    tgt_dist = 180 * 1800;
    tgt_angle = 0;

    % ego
    ego_v = v_max;
    ego_accl = 0;
    ego_w = 0;
    ego_alpha = 0;
    ego_dist = 0;
    ego_state = int8(0);

    % radius = 90.5;
    radius = 45;
    alphaTemp = (ego_v / radius);
    sla.base_alpha = alphaTemp;
    % sla.base_time = 0.18627166748046875;
    sla.base_time = 0.0731249999999984;

    sla.limit_time_count = sla.base_time * 2 / dt;
    sla.pow_n = 4;
    sla.state = 0;
    sla.counter = int32(0);

    tmp_x = single(0);
    tmp_y = single(0);
    tmp_w = single(0);
    tmp_theta = single(0);
    tmp_dist = single(0);

    tmp_x_list = zeros(100, 1);
    tmp_y_list = zeros(100, 1);

    for i = 2:1:sla.limit_time_count + 1
        tmp_alpha = alphaTemp * calc_neipire(dt * (i - 1), sla.base_time, sla.pow_n);
        tmp_w = tmp_w + tmp_alpha * dt;
        tmp_theta = tmp_theta + tmp_w * dt;
        tmp_x = tmp_x + ego_v * cos(tmp_theta) * dt;
        tmp_y = tmp_y + ego_v * sin(tmp_theta) * dt;
        tmp_dist = tmp_dist + ego_v * dt;
        trajectory_shape_list(i).x = single(tmp_x);
        trajectory_shape_list(i).y = single(tmp_y);
        trajectory_shape_list(i).w = single(tmp_w);
        trajectory_shape_list(i).theta = single(tmp_theta);
        trajectory_shape_list(i).dist = single(tmp_dist);
        trajectory_shape_list_size = int32(i);
        tmp_x_list(i) = trajectory_shape_list(i).x;
        tmp_y_list(i) = trajectory_shape_list(i).y;
    end

    plot(tmp_x_list, tmp_y_list, 'LineWidth', 2);
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
elseif test_mode == 2

    % tgt
    v_max = 0;
    end_v = 0;
    accl = 0;
    decel = 0;
    tgt_angle = 90 * 2 * pi / 360;

    param_alpha = -10;
    w_max = 2.5;
    end_w = 0.1;
    tgt_dist = 0;

    if param_alpha < 0
        w_max = w_max * (-1);
        tgt_angle = tgt_angle * (-1);
        end_w = end_w * (-1);
    end

    % ego
    ego_v = 0;
    ego_accl = 0;
    ego_w = 0;
    ego_alpha = 0;
    ego_dist = 0;
    ego_state = int8(0);

elseif test_mode == 3
    % tgt
    v_max = -100;
    end_v = -10;
    accl = -1000;
    decel = 500;
    w_max = 0;
    end_w = 0;
    param_alpha = 0;
    tgt_dist = -10;
    tgt_angle = 0;

    % ego
    ego_v = 0;
    ego_accl = 0;
    ego_w = 0;
    ego_alpha = 0;
    ego_dist = 0;
    ego_state = int8(0);

    PhyBase.accl_limitter_x = [0, 2500, 3500, 4500, 6500];
    PhyBase.accl_limitter_gain = [1, 1, 0.8, 0.6, 0.2];

    radius = 120;
    alphaTemp = (ego_v / radius);
    sla.base_alpha = alphaTemp;
    sla.base_time = 0.08233642578125;
    sla.limit_time_count = sla.base_time * 2 / dt;
    sla.pow_n = 4;
    sla.state = 0;
    sla.counter = int32(1);

elseif test_mode == 4
    % tgt
    v_max = 1400;
    end_v = v_max;
    accl = 28000;
    decel = -20000;
    w_max = 0;
    end_w = 0;
    param_alpha = 0;
    tgt_dist = 180 * 1800;
    tgt_angle = 0;

    % ego
    ego_v = v_max;
    ego_accl = 0;
    ego_w = 0;
    ego_alpha = 0;
    ego_dist = 0;
    ego_state = int8(0);

    % radius = 90.5;
    radius = 68;

    tgt_angle = 180.0 * pi / 180;
    param_alpha = (2 * ego_v^2 / (radius^2 * tgt_angle / 2));

    tgt_angle = 45.0 * pi / 180;
    param_alpha = (2 * ego_v^2 / (radius^2 * tgt_angle / 2));

    w_max = 250000;
    end_w = 0.1;
    tgt_dist = 0;

    if param_alpha < 0
        w_max = w_max * (-1);
        tgt_angle = tgt_angle * (-1);
        end_w = end_w * (-1);
    end

    sla.limit_time_count = sla.base_time * 2 / dt;
    sla.pow_n = 4;
    sla.state = 0;
    sla.counter = int32(0);

    tmp_x = single(0);
    tmp_y = single(0);
    tmp_w = single(0);
    tmp_theta = single(0);
    tmp_dist = single(0);

    tmp_x_list = zeros(100, 1);
    tmp_y_list = zeros(100, 1);

    for i = 2:1:sla.limit_time_count + 1
        tmp_alpha = alphaTemp * calc_neipire(dt * (i - 1), sla.base_time, sla.pow_n);
        tmp_w = tmp_w + tmp_alpha * dt;
        tmp_theta = tmp_theta + tmp_w * dt;
        tmp_x = tmp_x + ego_v * cos(tmp_theta) * dt;
        tmp_y = tmp_y + ego_v * sin(tmp_theta) * dt;
        tmp_dist = tmp_dist + ego_v * dt;
        trajectory_shape_list(i).x = single(tmp_x);
        trajectory_shape_list(i).y = single(tmp_y);
        trajectory_shape_list(i).w = single(tmp_w);
        trajectory_shape_list(i).theta = single(tmp_theta);
        trajectory_shape_list(i).dist = single(tmp_dist);
        trajectory_shape_list_size = int32(i);
        tmp_x_list(i) = trajectory_shape_list(i).x;
        tmp_y_list(i) = trajectory_shape_list(i).y;
    end

    plot(tmp_x_list, tmp_y_list, '-o');
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
end

predict_list = [1:10];

cd(home);
