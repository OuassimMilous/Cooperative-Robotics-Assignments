function [combinedx] = ComputeCooperativeXdot(desiredx1,desiredx2,x1,x2,h1,h2)

    m1_0=1;
    m2_0=1;
    m1 = m1_0 + norm([desiredx1 - x1]);
    m2 = m2_0 + norm([desiredx2 - x2]);
    xw =  (1/(m1 + m2))*(m1*x1 + m2*x2);
    C = [h1 h2];
    [U,D,V] = svd(C);
    C_pinv = V * pinv(D)*U';
    

   
    combinedx = [h1, zeros(6); zeros(6), h2] * [xw;xw] +  [h1, zeros(6); zeros(6), h2] * C_pinv * C* [desiredx1  ; desiredx2];

end
