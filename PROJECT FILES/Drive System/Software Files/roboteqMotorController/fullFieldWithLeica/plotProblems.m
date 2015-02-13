% Matlab visualization of MPC

horizon = 100;

aTo = importdata('aT', ':');
avo = importdata('av', ':');
aXo = importdata('aX', ':');
aYo = importdata('aY', ':');
cTo = importdata('cT', ':');
lvo = importdata('lv', ':');
aT = aTo.data;
av = avo.data;
aX = aXo.data;
aY = aYo.data;
cT = cTo.data;
lv = lvo.data;

x = zeros(horizon,1);
y = zeros(horizon,1);
xG = zeros(horizon,1);
yG = zeros(horizon,1);
xT = zeros(horizon,1);
yT = zeros(horizon,1);
xA = zeros(horizon/2,1);
yA = zeros(horizon/2,1);
time = 0;

for i = 1:size(lv,1)
    for j = 1:horizon/2
        if(av(i)==0) 
            x(j) = 0;
            y(j) = lv(i)*j/10;
        else
            x(j) = -lv(i)/av(i)*(1-cos(av(i)*j/10));
            y(j) = lv(i)/av(i)*sin(av(i)*j/10);
        end
        xA(j) = cos(aT(i))*x(j)-sin(aT(i))*y(j);
        yA(j) = sin(aT(i))*x(j)+cos(aT(i))*y(j);
        xA(j) = xA(j)+aX(i);
        yA(j) = yA(j)+aY(i);
    end
    for j = 1:horizon
        [xT(j),yT(j)] = desired(j*.05+cT(i));
        xG(j) = cos(aT(i+1))*xT(j)+sin(aT(i+1))*yT(j);
        yG(j) = -sin(aT(i+1))*xT(j)+cos(aT(i+1))*yT(j);
        xG(j) = xG(j)-aX(i+1);
        yG(j) = yG(j)-aY(i+1);
    end
%     [xA,yA]
    [lv(i),av(i),aX(i),aY(i)]
%     [xG,yG]
%     [x,y]
    plot(xA,yA,xT,yT);
    axis([-200 100 -10 350]);
    legend('chosen','goal');
    pause(.01);
end