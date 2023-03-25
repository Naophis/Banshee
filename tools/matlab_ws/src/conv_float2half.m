function [res_half, res_single] = conv_float2half(from_single, from_half)
    res_half = half(from_single);
    res_single = single(from_half);
end
