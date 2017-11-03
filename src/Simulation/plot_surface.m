fileID = fopen('plot_data.txt','r');
formatSpec = '%f';
jointData = fscanf(fileID,formatSpec,[3 Inf]);
fclose (fileID);
% size(jointData)
% jointData(1,1:end-1)
x = jointData(1,1:end-1);
 y = jointData(2, 1:end-1);
 z = jointData(3, 1:end-1);
[X,Y,Z] = meshgrid(x, y, z);
%  surf(X,Y,Z)
mesh(X,Y,Z)
% scatter3(x,y,z)
