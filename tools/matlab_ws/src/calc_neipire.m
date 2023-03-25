function output = calc_neipire(t, s, N)
    z = 1;
    P = 0;
    Q = 0;
    t = t / s;
    P = power((t - z), N - z);
    Q = P * (t - z);
    output = -N * P / ((Q - z) * (Q - z)) * power(exp(1), z + z / (Q - z)) / s;
end
