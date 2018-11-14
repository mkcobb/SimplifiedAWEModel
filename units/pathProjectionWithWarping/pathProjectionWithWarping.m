function s = pathProjectionWithWarping(s0,x,y,w,h,sDist,wFactor)
% function that finds the solution to 
% s = argmin(distance from current point to warped path)
% subject to: s_min < s < s_max
% where s_min = s0, and s_max = s0+sDist

sVec = linspace(s0,s0+sDist);

distances = sqrt(sum(([x y]-pathPositionWithWarping(sVec,w,h,wFactor)).^2,2));

[~,idx] = min(distances);
 
s=sVec(idx);

% options = optimoptions('fmincon','Display','off',...
%     'OptimalityTolerance',1e-3,'StepTolerance',1e-4);
% s = fmincon(fun,s0,[],[],[],[],s0,s0+sDist,[],options);
s = rem(s,1);
% if abs(s-s0)<1e-3
%     s = s0;
% end
end