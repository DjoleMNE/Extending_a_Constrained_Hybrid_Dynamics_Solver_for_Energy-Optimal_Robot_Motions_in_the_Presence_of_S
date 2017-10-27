clc
clear all

%Initial conditions:
Theta1=0;
% Theta1=3*pi/2;
Theta2=pi/6;

L1=0.4;
L2=0.4;

fileID = fopen('joint_poses.txt','r');
formatSpec = '%f';
jointPoses = fscanf(fileID,formatSpec,[2 Inf]);
fclose (fileID);
jointPoses = jointPoses';
counter = size(jointPoses,1);
cla

% size(jointPoses)
for i = 1:counter

    Theta1 = jointPoses(i,1)+3*pi/2;
    Theta2 = jointPoses(i,2);

    %Calculate the location of the middle two joints

    pointl1 = [L1*cos(Theta1) ; L1*sin(Theta1)];
    pointl2 = pointl1 + [L2*cos(Theta1+Theta2);
                        L2*sin(Theta1+Theta2)];

    %Plot
    axis([-2 2 -1 1])
    axis square
    line([0,pointl1(1)],[0,pointl1(2,1)])
    hold on
    line([pointl1(1),pointl2(1)],[pointl1(2,1),pointl2(2,1)])
    plot(pointl2 (1),pointl2 (2),'O')
    pause(.1)

end
