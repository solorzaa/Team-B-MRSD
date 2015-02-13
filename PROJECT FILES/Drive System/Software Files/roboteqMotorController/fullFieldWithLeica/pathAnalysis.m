desiredPath = [0 1 2 3 4 5 6 7 8 9 10;
               0 2 4 6 8 10 12 14 16 18 20;
               0 -1 -2 -3 -4 -5 -6 -7 -8 -9 -10;
               0 1 2 3 4 5 6 7 8 9 10];
shift = 5;
desiredPath = desiredPath + shift;
botPosition = [10 15];
botAngle = pi/4;

%Plot Settings
hold on
axis manual
t = 0:1:10;
botAxis = [botPosition(1) + cos(botAngle)*t;
           botPosition(2) + sin(botAngle)*t;
           botPosition(1) + cos(botAngle)*t;
           botPosition(2) - sin(botAngle)*t];
plot(botAxis(1,:), botAxis(2,:))
plot(botAxis(3,:), botAxis(4,:))

transformedPath = zeros(4,11);

%Translation
translatedPath(1,:) = desiredPath(1,:) - botPosition(1); 
translatedPath(2,:) = desiredPath(2,:) - botPosition(2);
translatedPath(3,:) = desiredPath(3,:) - botPosition(1); 
translatedPath(4,:) = desiredPath(4,:) - botPosition(2);

%Rotation
transformedPath(1,:) = translatedPath(1,:)*cos(botAngle) - translatedPath(2,:)*sin(botAngle);
transformedPath(2,:) = translatedPath(1,:)*sin(botAngle) + translatedPath(2,:)*cos(botAngle);
transformedPath(3,:) = translatedPath(3,:)*cos(botAngle) - translatedPath(4,:)*sin(botAngle);
transformedPath(4,:) = translatedPath(3,:)*sin(botAngle) + translatedPath(4,:)*cos(botAngle);

plot(desiredPath(1,:), desiredPath(2,:), 'r')
plot(desiredPath(3,:), desiredPath(4,:), 'r')
plot(transformedPath(1,:), transformedPath(2,:))
plot(transformedPath(3,:), transformedPath(4,:))
axis([-25 25 -25 25])