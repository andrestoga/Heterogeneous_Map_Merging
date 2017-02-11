ranges_path = '~/ros_Map_Merging/src/poles_detection/src/output_range.txt'

file = fopen(ranges_path, 'r');

formatSpec = '%f';
sizeA = [1 Inf];

A = fscanf(file, formatSpec, sizeA);

fclose(file);

tmp = size(A);

tmp(2)

X = 1:tmp(2);

% Y = A()

plot(X, A);