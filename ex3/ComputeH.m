function [H] = ComputeH(pandaArm)
[U,D,V] = svd(pandaArm.wJt);
J_pinv = V * pinv(D)*U';
H = pandaArm.wJt * J_pinv;
end