function ego = debug(default_ego, clock, ego_z, test_mode, sim_time)

    ego = default_ego;

    persistent ideal_x_list;
    persistent ideal_y_list;
    persistent slipped_x_list;
    persistent slipped_y_list;
    persistent kanayama_x_list;
    persistent kanayama_y_list;
    persistent i;

    if isempty(ideal_x_list)
        ideal_x_list = zeros(int32(sim_time / (0.001/4)), 1);
        ideal_y_list = zeros(int32(sim_time / (0.001/4)), 1);
        slipped_x_list = zeros(int32(sim_time / (0.001/4)), 1);
        slipped_y_list = zeros(int32(sim_time / (0.001/4)), 1);
        kanayama_x_list = zeros(int32(sim_time / (0.001/4)), 1);
        kanayama_y_list = zeros(int32(sim_time / (0.001/4)), 1);
        i = int32(0);
    end

    if clock > 0
        ego = ego_z;

        i = int32(1) + i;

        if test_mode == 1
            ideal_x_list(i) = ego.ideal_point.x;
            ideal_y_list(i) = ego.ideal_point.y;
            slipped_x_list(i) = ego.slip_point.x;
            slipped_y_list(i) = ego.slip_point.y;
            kanayama_x_list(i) = ego.kanayama_point.x;
            kanayama_y_list(i) = ego.kanayama_point.y;
        elseif test_mode == 4
            ideal_x_list(i) = ego.ideal_point.x;
            ideal_y_list(i) = ego.ideal_point.y;
            slipped_x_list(i) = ego.slip_point.x;
            slipped_y_list(i) = ego.slip_point.y;
            kanayama_x_list(i) = ego.kanayama_point.x;
            kanayama_y_list(i) = ego.kanayama_point.y;
        end
        if sim_time == clock
            subplot(2, 1, 1);
            plot(ideal_x_list, ideal_y_list, 'b:');
            hold on;
            plot(slipped_x_list, slipped_y_list, '-r');
           
            % xlim([-90 270]);
            % ylim([-90 270]);
            xlim([-90 540]);
            ylim([-90 540]);

            plot([-90 270], [0 0], 'k:');
            plot([-90 270], [180 180], 'k:');
            
            plot([0 0],[-90 270], 'k:');
            plot([180 180],[-90 270] ,'k:');
            
            plot([0 360],[-90 270] ,'k:');
            plot([0 360],[270 -90] ,'k:');
            
            plot([-90 270], [84 84], 'r:');
            plot([-90 270], [96 96], 'r:');
            
            plot([84 84], [-90 270], 'r:');
            plot([96 96], [-90 270], 'r:');

            hold off;

            subplot(2, 1, 2);
            plot(kanayama_x_list, kanayama_y_list, '-k');
            hold on;
            plot(slipped_x_list, slipped_y_list, '-r');
           
            xlim([-90 270]);
            ylim([-90 270]);

            plot([-90 270], [0 0], 'k:');
            plot([-90 270], [180 180], 'k:');
            
            plot([0 0],[-90 270], 'k:');
            plot([180 180],[-90 270] ,'k:');
            
            plot([0 360],[-90 270] ,'k:');
            plot([0 360],[270 -90] ,'k:');
            
            plot([-90 270], [84 84], 'r:');
            plot([-90 270], [96 96], 'r:');
            
            plot([84 84], [-90 270], 'r:');
            plot([96 96], [-90 270], 'r:');
            hold off;

        end

        if ego.sla_param.limit_time_count >=  ego.sla_param.counter
            % ego.v = ego.kanayama_point.v;
            % ego.w = ego.kanayama_point.w;
        end
    end

end
