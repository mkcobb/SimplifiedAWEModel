function p = pathPosition(s,w,h)
x = (w/2)*cos((2*s+3/2)*pi);
y = h*cos((2*s+3/2)*pi).*sin((2*s+3/2)*pi);
p =[x' y'];
end