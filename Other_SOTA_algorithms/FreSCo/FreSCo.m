function [this_part_pol_log, key] = FreSCo(ptcloud)
%     gridStep = 0.75; % cubic grid downsampling(m). 
%     ptcloud = pcdownsample(ptcloud, 'gridAverage', gridStep);
    points = ptcloud.Location;
    
    points = imresize(points, [101, 101], 'bilinear');
    input_fft = fft2(points);
    input_amp = abs(fftshift(input_fft));
    low_freq_part_mat = applyGaussian(input_amp(31:71, 31:71));
    this_part_pol = ImToPolar(low_freq_part_mat, 0, 1, 20, 120);
    this_part_pol_log = log(this_part_pol);

    this_key = mean((this_part_pol_log), 2); 
    fresco_mean = mean(this_part_pol_log, 'all');
    this_key = this_key ./ fresco_mean; % normalization
    this_std = std(this_part_pol_log, 0, 2) ./ fresco_mean;
    key = [this_key; this_std];
end

function [img_out] = applyGaussian(img_in)
    sigma = 1;  %设定标准差值，该值越大，滤波效果（模糊）愈明显
    window = double(uint8(3*sigma)*2 + 1);  %设定滤波模板尺寸大小
    %fspecial('gaussian', hsize, sigma)产生滤波掩模
    G = fspecial('gaussian', window, sigma);
    img_out = imfilter(img_in, G, 'conv','replicate','same');
end