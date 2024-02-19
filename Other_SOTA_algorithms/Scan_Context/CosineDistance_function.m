function dist = CosineDistance_function(ScanContext_1, ScanContext_2)
% 输入：两个ScanContext
% 输出：相似性差异距离
% 注意：受同一位置下采集方向的不同，需根据旋转不变性，对第一个矩阵循环平移Sectors次，每次得到的矩阵都与另一个矩阵进行相似度距离测量，最终找出最佳的那个。
sectors_size = size(ScanContext_1, 2); % 通过传入的scan context矩阵列数得到sector的数目
similarity_each_shift = zeros(1,sectors_size); % 每一次平移的差异性比较（旋转不变性）都保存，方便后面找出相似性最大时的对应的最小差异距离
for ith_shift = 1 : sectors_size
    OneStep_length = 1; % 每次平移的步长
    ScanContext_1 = circshift(ScanContext_1, OneStep_length, 2); %OneScanContext矩阵按列循环右平移一步

    ithShift_CosSimilarity_sum = 0; % 每次平移后，两个scan context的所有非零列余弦和
    NonZeroCols_in_sectors = 0; % 两个scan context公共非零列统计
    for sectors_ith = 1 : sectors_size
        FirstScanContext_ithCols = ScanContext_1(:, sectors_ith); % 获取OneScanContext对应的列数据
        SecondScanContext_ithCols = ScanContext_2(:, sectors_ith); % 获取TwoScanContext对应的列数据

        if(~any(FirstScanContext_ithCols) || ~any(SecondScanContext_ithCols))
            continue; % 对应的两列，只要有一个出现全零，就不参与计算
        end

        % CosSimilarity_ithCols = dot(FirstScanContext_ithCols, SecondScanContext_ithCols) / (norm(FirstScanContext_ithCols)*norm(SecondScanContext_ithCols)); % 求对应非零列的余弦距离。
        CosSimilarity_ithCols = pdist([FirstScanContext_ithCols'; SecondScanContext_ithCols'],'cosine'); % cosine, euclidean, cityblock, chebychev,minkowski.
        ithShift_CosSimilarity_sum = ithShift_CosSimilarity_sum +CosSimilarity_ithCols; % 所有对应非零列的余弦距离进行累加。
        
        NonZeroCols_in_sectors = NonZeroCols_in_sectors + 1; % 统计共有多少对非零列。
    end

    similarity_each_shift(ith_shift) = ithShift_CosSimilarity_sum / NonZeroCols_in_sectors; % 保存每次平移下两scan context的相似性差异
end

% dist = 1 - max(similarity_each_shift); %找出最大相似度，求最小差异距离。
dist = min(similarity_each_shift); %找出最大相似度，求最小差异距离。
end