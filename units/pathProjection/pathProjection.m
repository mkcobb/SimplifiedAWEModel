function s = pathProjection(s0,x,y,w,h,sDist)
fun = @(s) norm([x y]-pathPosition(s,w,h));
s = fminbnd(fun,s0,s0+sDist);

s = rem(s,1);
if abs(s-s0)<0.001
    s = s0;
end
end