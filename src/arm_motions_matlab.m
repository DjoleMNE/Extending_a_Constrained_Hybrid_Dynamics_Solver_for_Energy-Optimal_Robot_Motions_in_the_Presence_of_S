%% Ben Goldberg - Inverse Kinematics of a Robot Arm
clc
clear all

%Initial conditions:
Theta1=0;
Theta2=pi/6;
% Theta3 = pi/6;
L1=-0.4;
L2=-0.4;
% L3=1;

dt =.01; %time step
counter = 0;

fileID = fopen('joint_poses.txt','r' );
formatSpec = '%f';
jointPoses = fscanf(fileID,formatSpec,[2 Inf]);
fclose (fileID);
jointPoses = jointPoses';
jointPoses
% jointPoses(1,1)
cla

% size(jointPoses)
for i = 0:dt:1*0.09999999 %End time should be adjusted for the desired accuracy
    
Theta1 = jointPoses(counter+1,1);
Theta2 = jointPoses(counter+1,2);

xo = [L1*cos(Theta1)+L2*cos(Theta1+Theta2);
      L2*sin(Theta1)+L2*sin(Theta1+Theta2)];


%Calculate the location of the middle two joints

pointl1 = [L1*cos(Theta1) ; L1*sin(Theta1)];
pointl2 = pointl1 + [L2*cos(Theta1+Theta2);
                    L2*sin(Theta1+Theta2)];

%Plot

if (mod(counter,1)==0) %plots every ...iterations
axis([-1 1 -0.4 0.4])
axis square
line([0,pointl1(1)],[0,pointl1(2,1)])
hold on
line([pointl1(1),pointl2(1)],[pointl1(2,1),pointl2(2,1)])
line([pointl2(1),xo(1)],[pointl2(2,1),xo(2,1)])
plot(xo(1),xo(2),'o')
pause(.1)
end

counter = counter +1;
end