function solIsUnique = solIsUnique(f,lambda)
designSpaceDimension = numel(f);

lambda = [lambda.lower;lambda.upper;lambda.eqlin;lambda.ineqlin];
solIsUnique = false;
if sum(lambda>1e-10)==designSpaceDimension
    solIsUnique = true;
end
end