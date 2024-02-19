function ring_key = ScanContext2RingKey_function(ScanContext)
% 输入：rings_num*sectors_num大小的scan context描述子矩阵
% 输出：1*rings_num大小的ringkey矩阵
rings_num = size(ScanContext, 1);
sectors_num = size(ScanContext, 2);

ring_key = zeros(1, rings_num);
for rings_ith=1:rings_num
    ithRing = ScanContext(rings_ith,:);
    NonZerosNum_ithRing = nnz(~ithRing); % 非零元素的数目
    ring_key(rings_ith) = (sectors_num - NonZerosNum_ithRing)/sectors_num;
end

% d = zeros(rings_num,sectors_num);
% wgt = ones(sectors_num,1);
% id0 = find(ScanContext == 0);
% idn0 = find(ScanContext ~= 0);
% d(id0) = 1;
% d(idn0) = 0;
% ring_key = (d * wgt) / sectors_num;
% ring_key = ring_key';

end