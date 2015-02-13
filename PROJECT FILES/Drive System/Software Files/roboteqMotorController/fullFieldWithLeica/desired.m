function [x,y] =  desired(t)
    if t < 36
        x=0;
        y=6*t;
    elseif t<92.5487
        x = (6*12)*cos((t-36)/56.5487*3*pi/2)-(6*12);
        y = (6*12)*sin((t-36)/56.5487*3*pi/2)+(18*12);
    elseif t<116.5487
        x = -(6*12)+(6*(t-92.5487));
        y = 12*12;
    else 
        x = 6*12;
        y = 12*12;
    end
end