function s = pathProjectionWithWarping(s0,x,y,w,h,sDist,wFactor)
% function that finds the solution to 
% s = argmin(distance from current point to warped path)
% subject to: s_min < s < s_max
% where s_min = s0, and s_max = s0+sDist

fun = @(s) norm([x y]-pathPositionWithWarping(s,w,h,wFactor));
s = fminbnd(fun,s0,s0+sDist);
s = rem(s,1);
if abs(s-s0)<0.001
    s = s0;
end
end