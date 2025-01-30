function [H] = ComputeH(pandaArm)
[U,D,V] = svd(pandaArm.bJt);
J_pinv = V * pinv(D)*U';
H = pandaArm.bJt * J_pinv;
end