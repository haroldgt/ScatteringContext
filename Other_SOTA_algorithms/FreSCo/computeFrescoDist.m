function [best_offset, fresco_dist] = computeFrescoDist(fresco1, fresco2)
    fresco_dist = 1e10;
    best_offset = 0;
    for angle_offset = 0:59 % since the fft img is symmetrical wrt the center point
        shifted_fresco2 = circleShift(fresco2, angle_offset);
        % log scale seems resulting better angle estimation
        corr = computeL1Dist(log(fresco1), log(shifted_fresco2)); % L1 dist 
        if corr < fresco_dist
            fresco_dist = corr;
            best_offset = angle_offset;
        end
    end
end

function [img_out] = circleShift(img_in, offset)
    rows = size(img_in, 1);
    cols = size(img_in, 2);
    
    img_out = zeros(rows, cols);
    
    for col_idx = 1:cols
        corr_col_idx = col_idx + offset;
        if corr_col_idx > cols
            corr_col_idx = corr_col_idx - cols;
        elseif corr_col_idx <= 0
            corr_col_idx = corr_col_idx + cols;
        end
        
        img_out(:, col_idx) = img_in(:, corr_col_idx);
    end
end

function [dist] = computeL1Dist(img1, img2)
    diff = img1 - img2;
    dist = sum(sum(abs(diff)));
end