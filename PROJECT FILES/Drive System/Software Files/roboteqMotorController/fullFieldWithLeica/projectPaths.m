linearVelocity = 0:1:5;
angularVelocity = -1:0.1:1;

t = 0:0.1:1;

for i = 1:size(linearVelocity,2)
    for j = 1:size(angularVelocity,2)

            pathX = -(linearVelocity(i) / angularVelocity(j)) * (1-cos(angularVelocity(j)*t));
            pathY = (linearVelocity(i) / angularVelocity(j)) * sin(angularVelocity(j)*t);
        
            hold on
            plot(pathX,pathY)

    end
end

