function [Recall, Precision] = RecallandPrecision(loop_gt, detection_result)
%函数功能：计算检测的召回率和精度。
%输入：exist_loop_gt --- 是否存在真实回环。nx1
%      detection_result --- 是否检测到回环，该回环是否正确。nx2

    a = loop_gt.*detection_result(:,1); % nx1 .* nx1
    Recall = length(find(a == 1))/length(find(loop_gt == 1)); % 计算召回率。
    b = detection_result(:,1).*detection_result(:,2); % nx1 .* nx1
    if ((Recall == 0 ) || (length(find(detection_result(:,1) == 1)) == 0))
        Precision = 1;
    else
        Precision = length(find(b == 1))/length(find(detection_result(:,1) == 1)); % 计算精度。
    end
end