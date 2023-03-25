function [l_start, l_end] = plot_slalom(turn_mode, tmp_x_list, tmp_y_list, tmp_w_list, ...
        tmp_x, tmp_y, target_angle, is_dia_mode, fig_num, wall_off_offset, wall_off_offset_dia)
    
    Normal = 0; Large = 1; Orval = 2; Dia45 = 3; Dia135 = 4; Dia90 = 5; Dia180 = 6; Dia45_2 = 7; Dia135_2 = 8;
    output = 0;
    fig1 = figure('Name',fig_num);
    clf(fig1);
    l_start = 0;
    l_end = 0;

    classic = 180;
    half = 90;
    cell_size = half;

    LineWidth = 2;
    if turn_mode == Normal
        [l_start, l_end] = calc_offset_dist(tmp_x_list, tmp_y_list, cell_size/2 - wall_off_offset, cell_size/2 - wall_off_offset, target_angle);
        l_start = l_start + wall_off_offset;
        plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
        hold on;
        plot([-wall_off_offset l_start], [0 0], 'LineWidth', LineWidth);
        l_end_x0 = tmp_x_list(end) + l_start;
        l_end_x1 = tmp_x_list(end) + l_start;
        l_end_y0 = tmp_y_list(end);
        l_end_y1 = tmp_y_list(end) + l_end;
        plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
        hold off;

    elseif turn_mode == Large
        [l_start, l_end] = calc_offset_dist(tmp_x_list, tmp_y_list, cell_size - wall_off_offset, cell_size - wall_off_offset, target_angle);
        l_start = l_start + wall_off_offset;
        plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
        hold on;
        plot([-wall_off_offset l_start], [0 0], 'LineWidth', LineWidth);
        l_end_x0 = tmp_x_list(end) + l_start;
        l_end_x1 = tmp_x_list(end) + l_start;
        l_end_y0 = tmp_y_list(end);
        l_end_y1 = tmp_y_list(end) + l_end;
        plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
        hold off;

    elseif turn_mode == Orval
        l_start = 10 + wall_off_offset;
        l_end = l_start + tmp_x_list(end) - wall_off_offset;
        plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
        hold on;
        plot([-wall_off_offset l_start], [0 0], 'LineWidth', LineWidth);
        l_end_x0 = tmp_x_list(end) + l_start;
        l_end_x1 = tmp_x_list(end) +l_start - l_end;
        l_end_y0 = tmp_y_list(end);
        l_end_y1 = tmp_y_list(end);
        plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
        hold off;
    elseif turn_mode == Dia45 || turn_mode == Dia45_2

        if ~is_dia_mode
            [l_start, l_end] = calc_offset_dist(tmp_x_list, tmp_y_list, cell_size - wall_off_offset_dia * sin(pi / 4), cell_size/2 - wall_off_offset_dia * sin(pi / 4), target_angle);
            %l_start = l_start + wall_off_offset;
            plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
            hold on;
            plot([-wall_off_offset l_start], [0 0], 'LineWidth', LineWidth);
            l_end_x0 = tmp_x_list(end) + l_start;
            l_end_x1 = tmp_x_list(end) + l_start + l_end * sin(pi / 4);
            l_end_y0 = tmp_y_list(end);
            l_end_y1 = tmp_y_list(end) + l_end * sin(pi / 4);
            plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
            hold off;
            l_start = l_start + wall_off_offset;
        else
            l_start = (cell_size/2 - tmp_x_list(end)) / sin(pi / 4);
            l_end = cell_size - tmp_y_list(end) - l_start * sin(pi / 4) - wall_off_offset;

            plot(tmp_x_list + l_start * cos(pi / 4), tmp_y_list + l_start * sin(pi / 4), 'LineWidth', LineWidth);
            hold on;
            l_start_x1 = l_start * sin(pi / 4);
            l_start_y1 = l_start * sin(pi / 4);
            plot([-wall_off_offset_dia * cos(pi / 4) l_start_x1], [-wall_off_offset_dia * sin(pi / 4) l_start_y1], 'LineWidth', LineWidth);
            l_end_x0 = tmp_x_list(end) + l_start * sin(pi / 4);
            l_end_x1 = tmp_x_list(end) + l_start * sin(pi / 4);
            l_end_y0 = tmp_y_list(end) + l_start * sin(pi / 4);
            l_end_y1 = tmp_y_list(end) + l_start * sin(pi / 4) + l_end;
            plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
            hold off;

            l_start = l_start + wall_off_offset_dia;
        end

    elseif turn_mode == Dia135|| turn_mode == Dia135_2

        if ~is_dia_mode
            [l_start, l_end] = calc_offset_dist(tmp_x_list, tmp_y_list, cell_size/2 + wall_off_offset_dia * sin(pi / 4), cell_size - wall_off_offset_dia * sin(pi / 4), target_angle);
            plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
            hold on;
            plot([-wall_off_offset l_start], [0 0], 'LineWidth', LineWidth);
            l_end_x0 = tmp_x_list(end) + l_start;
            l_end_x1 = tmp_x_list(end) + l_start - l_end * sin(pi / 4);
            l_end_y0 = tmp_y_list(end);
            l_end_y1 = tmp_y_list(end) + l_end * sin(pi / 4);
            plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
            hold off;
            l_start = l_start + wall_off_offset;
        else
            l_start = (cell_size - tmp_y_list(end)) / sin(pi / 4);
            l_end = abs(-cell_size/2 - tmp_x_list(end) - l_start * sin(pi / 4) + wall_off_offset);

            plot(tmp_x_list + l_start * sin(pi / 4), tmp_y_list + l_start * sin(pi / 4), 'LineWidth', LineWidth);
            hold on;
            l_start_x1 = l_start * sin(pi / 4);
            l_start_y1 = l_start * sin(pi / 4);
            plot([-wall_off_offset_dia * cos(pi / 4) l_start_x1], [-wall_off_offset_dia * cos(pi / 4) l_start_y1], 'LineWidth', LineWidth);
            l_end_x0 = tmp_x_list(end) + l_start * sin(pi / 4);
            l_end_x1 = tmp_x_list(end) + l_start * sin(pi / 4) - l_end;
            l_end_y0 = tmp_y_list(end) + l_start * sin(pi / 4);
            l_end_y1 = tmp_y_list(end) + l_start * sin(pi / 4);
            plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
            hold off;
            l_start = l_start + wall_off_offset_dia;
        end

    elseif turn_mode == Dia90
        [l_start, l_end] = calc_offset_dist(tmp_x_list, tmp_y_list, cell_size/2 * sqrt(2) - wall_off_offset_dia, cell_size/2 * sqrt(2) - wall_off_offset_dia, target_angle);
        l_start = l_start + wall_off_offset_dia;
        plot(tmp_x_list + l_start, tmp_y_list, 'LineWidth', LineWidth);
        hold on;
        plot([-wall_off_offset_dia l_start], [0 0], 'LineWidth', LineWidth);
        l_end_x0 = tmp_x_list(end) + l_start;
        l_end_x1 = tmp_x_list(end) + l_start;
        l_end_y0 = tmp_y_list(end);
        l_end_y1 = tmp_y_list(end) + l_end;
        plot([l_end_x0 l_end_x1], [l_end_y0 l_end_y1], 'LineWidth', LineWidth);
        hold off;

    end

    if turn_mode == Dia90
        xlim([-40 200]);
        ylim([-40 200]);
        hold on;

        plot([-270 270], [0 0], 'k:');
        % plot([-270 270], [cell_size cell_size], 'k:');
        tmp_dist = cell_size/2 * sqrt(2);
        plot([-270 270], [tmp_dist tmp_dist], 'k:');

        plot([0 0], [-cell_size/2 270], 'k:');
        plot([tmp_dist tmp_dist], [-cell_size/2 270], 'k:');

        plot([-tmp_dist / 2 -tmp_dist / 2], [-cell_size 270], 'k:');
        plot([tmp_dist / 2 tmp_dist / 2], [-cell_size 270], 'k:');

        plot([-cell_size/2 360], [-cell_size/2 360], 'k:');

        hold off;
    elseif ~is_dia_mode
        xlim([-40 220]);
        ylim([-40 220]);
        hold on;

        plot([-cell_size/2 270], [0 0], 'k:');
        plot([-cell_size/2 270], [cell_size cell_size], 'k:');

        plot([0 0], [-cell_size/2 270], 'k:');
        plot([cell_size cell_size], [-cell_size/2 270], 'k:');

        plot([0 360], [-cell_size/2 270], 'k:');
        plot([0 360], [270 -cell_size/2], 'k:');

        plot([-cell_size/2 270], [84 84], 'r:');
        plot([-cell_size/2 270], [96 96], 'r:');

        plot([84 84], [-cell_size/2 270], 'r:');
        plot([96 96], [-cell_size/2 270], 'r:');

        hold off;
    else
        xlim([-120 cell_size]);
        ylim([-cell_size/2 270]);

        hold on;

        plot([-cell_size/2 270], [0 0], 'k:');
        plot([-cell_size/2 270], [cell_size cell_size], 'k:');

        plot([0 0], [-cell_size/2 270], 'k:');
        plot([cell_size cell_size], [-cell_size/2 270], 'k:');

        plot([-cell_size/2 -cell_size/2], [-cell_size 270], 'k:');
        plot([cell_size/2 cell_size/2], [-cell_size 270], 'k:');

        plot([-cell_size/2 360], [-cell_size/2 360], 'k:');
        plot([-cell_size/2 270], [270 -cell_size/2], 'k:');

        plot([-cell_size 6], [84 84], 'r:');
        plot([-cell_size 6], [96 96], 'r:');

        plot([6 6], [84 96], 'r:');

        plot([174 174], [-cell_size/2 270], 'r:');
        plot([186 186], [-cell_size/2 270], 'r:');

        plot([-cell_size/2 270], [264 264], 'r:');
        plot([-cell_size/2 270], [276 276], 'r:');

        hold off;
    end
    

end
