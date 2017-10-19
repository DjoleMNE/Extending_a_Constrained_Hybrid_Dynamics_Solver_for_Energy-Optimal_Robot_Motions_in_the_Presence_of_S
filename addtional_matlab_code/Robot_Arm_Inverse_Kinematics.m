%% Ben Goldberg - Inverse Kinematics of a Robot Arm



clc

clear all

close all



%Initial conditions:

Theta1=pi/6;
Theta2=pi/6;
Theta3 = pi/6;
L1=1;
L2=1;
L3=1;
W=[1 0 0; 0 1 0; 0 0 1]; %weighting matrix (identity all joints penalized equally
xee = [1;1]; %Desired end effector location    
dt =.01; %time step
counter = 0;



for i = 0:dt:2 %End time should be adjusted for the desired accuracy



%Jacobian (2x3) Partial xee with respect to Theta1,2,3

J= [-L1*sin(Theta1)-L2*sin(Theta1+Theta2)-L3*sin(Theta1+Theta2+Theta3),...
-L2*sin(Theta1+Theta2)-L3*sin(Theta1+Theta2+Theta3),...
-L3*sin(Theta1+Theta2+Theta3);...
L1*cos(Theta1)+L2*cos(Theta1+Theta2)+L3*cos(Theta1+Theta2+Theta3),...
L2*cos(Theta1+Theta2)+L3*cos(Theta1+Theta2+Theta3),...
L3*cos(Theta1+Theta2+Theta3)];

     

%Pseudoinverse given the Jacobian and Weighting matrix

Jpseudo = inv(W)*J'*inv(J*inv(W)*J');


%End - effector location

xo = [L1*cos(Theta1)+L2*cos(Theta1+Theta2)+L3*cos(Theta1+Theta2+Theta3);...
L2*sin(Theta1)+L2*sin(Theta1+Theta2)+L3*sin(Theta1+Theta2+Theta3)];



%Calculate the location of the middle two joints

pointl1 = [L1*cos(Theta1) ; L1*sin(Theta1)];
pointl2 = pointl1 + [L2*cos(Theta1+Theta2);L2*sin(Theta1+Theta2)];



%Plot

if (mod(counter,10)==0) %plots every 10 iterations
axis([-3 3 -3 3])
axis square
line([0,pointl1(1)],[0,pointl1(2,1)])
hold on
line([pointl1(1),pointl2(1)],[pointl1(2,1),pointl2(2,1)])
line([pointl2(1),xo(1)],[pointl2(2,1),xo(2,1)])
plot(xo(1),xo(2),'o')
pause(.1)

end



dist = (sqrt((xo(1)-xee(1))^2+(xo(2)-xee(2))^2)); %Distance between

%current EE location and Final EE location is used to normalize the error 
%value (x dot)



%Angular velocity of each angle:

thetadot = Jpseudo*((xee-xo)/dist);

theta1 = thetadot(1);

theta2 = thetadot(2,1);

theta3 = thetadot(3,1);



%Updtate each angle based on calculated angular velocity

Theta1 = Theta1 + dt*theta1;

Theta2 = Theta2 + dt*theta2;

Theta3 = Theta3 + dt*theta3;


counter = counter +1;

end






