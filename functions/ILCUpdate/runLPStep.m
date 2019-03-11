function [deltaUStar,deltaTStar] = runLPStep(xNominal,deltax0,F,alphaMat, gammaMat,stateReference,waypointPathIndices,trackingErrorTolerance,deltaDeltaUMax, deltaTMin,deltaTMax,deltaUNom, TimeStepsNominal)
%% Take in the alpha and gamma matrices, and state reference vector. Run linprog while only penalizing tracking at the waypoints. 
% Minimize time stepping sequence while staying within upper and lower
% bounds on deltaDeltaU and deltaT.

% The state reference in this case would be a vector with the number of
% elements equalling the number of path steps times the number of states.
% However the only elements we care about are the ones corresponding to the
% x,y positions at the waypoints

lengthOfStateVectorSequence = size(gammaMat,1);
numberOfPathSteps = size(gammaMat,2);

selectionVector = zeros(lengthOfStateVectorSequence,1);
selectionVector(4*waypointPathIndices-3) = 1;
selectionVector(4*waypointPathIndices-2) = 1;
%selectionMatrix = diag(selectionVector); %Generate the selection matrix to pick off waypoint information

selectionMatrix = zeros(2*numberOfPathSteps,lengthOfStateVectorSequence);
for i=1:length(waypointPathIndices)
   selectionMatrix(2*waypointPathIndices(i)-1,4*waypointPathIndices-3)=1;
   selectionMatrix(2*waypointPathIndices(i),4*waypointPathIndices-2)=1;
end

deltaTSummer = [zeros(numberOfPathSteps,1); ones(numberOfPathSteps,1)];

deltaUMinVec = -deltaDeltaUMax*ones(numberOfPathSteps,1)+deltaUNom;
deltaUMaxVec = deltaDeltaUMax*ones(numberOfPathSteps,1)+deltaUNom;
deltaTMinVec = (deltaTMin-1)*TimeStepsNominal;
deltaTMaxVec = (deltaTMax-1)*TimeStepsNominal;

f = deltaTSummer;
A = [selectionMatrix;-selectionMatrix]*[alphaMat gammaMat];
b = [selectionMatrix;-selectionMatrix]*(stateReference-xNominal-F*deltax0)+trackingErrorTolerance*ones(4*numberOfPathSteps,1);
Aeq=[];
beq=[];
lb = [deltaUMinVec; deltaTMinVec];
ub = [deltaUMaxVec; deltaTMaxVec];

deltaUStarDeltaTStarSequence = linprog(f,A,b,Aeq,beq,lb,ub);

deltaUStarSelector = [eye(numberOfPathSteps), zeros(numberOfPathSteps)];
deltaTStarSelector = [zeros(numberOfPathSteps), eye(numberOfPathSteps)];

deltaUStar = deltaUStarSelector*deltaUStarDeltaTStarSequence;
deltaTStar = deltaTStarSelector*deltaUStarDeltaTStarSequence;
end