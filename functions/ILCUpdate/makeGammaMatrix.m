function [gammaMat] = makeGammaMatrix(Ad,Bd,ACont, BCont, pathStepTimes,deltauNom)
% Generate the gamma matrix to capture the effect of changes in pathstep
% timing
Tnj = [diff(pathStepTimes) 0];
numStates = size(Ad,1);
N = numStates*length(Tnj);

% The beta matrix is the linearization of the lifted matrix with respect to
% the timesteps
betaMat = zeros(N,N/numStates);
for m=1:numStates:N
    currPathStep = ceil(m/numStates);
    for n=1:currPathStep
        cumulativeAd = eye(size(Ad,1));
        for i=n:currPathStep-1
           cumulativeAd = cumulativeAd*Ad(:,:,i);
        end
        
        if currPathStep==n
            betaMat(m:m+numStates-1,n) = Ad(:,:,n)*BCont(:,:,n);
        else
            %Need to check if ACont(:,:,n) is the right variable to be
            %using here
            betaMat(m:m+numStates-1,n) = (currPathStep-n)*ACont(:,:,n)*cumulativeAd*Bd(:,:,n)+cumulativeAd*Ad(:,:,n)*BCont(:,:,n);
        end
    end
end

%The gamma matrix comes from the linearization of the lifted system input-state relationship
% which gives x = xNominal+F*deltax0+alphaMat*deltaU+gammaMat*deltaT
gammaMat = betaMat*diag(deltauNom);
end

