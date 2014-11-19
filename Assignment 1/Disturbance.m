function disturbance = Disturbance( vector, value )
    disturbance = vector*sqrt(value)*randn(3,1);
end

