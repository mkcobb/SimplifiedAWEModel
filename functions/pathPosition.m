function p = pathPosition(s,w,h)
% Function to calculate the shape of the path, given the basis paramters, w
% and h.  The input s is the position along the path between 0 (start of
% path) and 1 (end of path).
x = (w/2)*cos((2*s+3/2)*pi);
y = h*cos((2*s+3/2)*pi).*sin((2*s+3/2)*pi);
p =[x' y'];
end