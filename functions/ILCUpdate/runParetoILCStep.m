function [deltaUStar] = runParetoILCStep(xNominal,deltax0,F,alphaMat, stateReference,waypointPathIndices,trackingCost,inputCost,deltaUMin,deltaUMax)
%% Take in the alpha and gamma matrices, and state reference vector. Run linprog while only penalizing tracking at the waypoints. 
% Minimize time stepping sequence while staying within upper and lower
% bounds on deltaDeltaU and deltaT.

% The state reference in this case would be a vector with the number of
% elements equalling the number of path steps times the number of states.
% However the only elements we care about are the ones corresponding to the
% x,y positions at the waypoints

lengthOfStateVectorSequence = size(alphaMat,1);
numberOfPathSteps = size(alphaMat,2);

%Selection matrix for picking off waypoint information.
selectionMatrix = zeros(2*numberOfPathSteps,lengthOfStateVectorSequence);
for i=1:length(waypointPathIndices)
   selectionMatrix(2*waypointPathIndices(i)-1,4*waypointPathIndices-3)=1;
   selectionMatrix(2*waypointPathIndices(i),4*waypointPathIndices-2)=1;
end

%Generating weighting matrices
Q = trackingCost*eye(2*numberOfPathSteps);
S = inputCost*eye(numberOfPathSteps);
% Creating parmeters for quadprog optimization
H = (selectionMatrix*alphaMat)'*Q*selectionMatrix*alphaMat+S;
H = round(H,10);
f = -(selectionMatrix*alphaMat)'*Q*(selectionMatrix*(stateReference-xNominal-F*deltax0));
A = [];
b = [];
Aeq = [];
beq = [];
lb = deltaUMin*ones(numberOfPathSteps,1);
ub = deltaUMax*ones(numberOfPathSteps,1);

deltaUStar = quadprog(H,f,A,b,Aeq,beq,lb,ub);
end