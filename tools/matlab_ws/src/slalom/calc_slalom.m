function output = calc_slalom(v, radius, Et, target)
    c = 0;
    R1 = 0;
    t1 = 0;
    dt = 0.001/64;

    while (c < 10000000)
        t1 = t1 + dt;
        R1 = 2 * v / radius * Et * t1;

        if (R1 >= target)
            output = t1;
            return;
        end

        c = c + 1;
    end

end
