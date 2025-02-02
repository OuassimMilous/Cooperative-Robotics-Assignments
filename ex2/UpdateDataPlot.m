function [ plt ] = UpdateDataPlot( plt, pandaArm, t, loop, mission )

% this function samples the variables contained in the structure pandaArm
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script


plt.t(:, loop) = t;
plt.q(:, loop) = pandaArm.ArmL.q;
plt.q_dot(:, loop) = pandaArm.ArmL.q_dot;
plt.q2(:, loop) = pandaArm.ArmR.q;
plt.q_dot2(:, loop) = pandaArm.ArmR.q_dot;

% Plot: desired object velocity
plt.xdot(:, loop) = pandaArm.ArmL.xdot.tool;
plt.qe_dot(:, loop) = pandaArm.ArmL.wJt(1:7) * pandaArm.ArmL.q;

plt.xdot2(:, loop) = pandaArm.ArmR.xdot.tool;
plt.qe_dot2(:, loop) = pandaArm.ArmR.wJt(8:14) * pandaArm.ArmR.q;

% Plot: manipulability task activation function
plt.A(1, loop) = mean(diag(pandaArm.ArmL.A.joints));
plt.A(2, loop) = mean(diag(pandaArm.ArmL.A.min));
plt.A(3, loop)= mean(diag(pandaArm.ArmL.A.tool));
plt.A(4, loop) = mean(diag(pandaArm.A.con));

plt.A2(1, loop) = mean(diag(pandaArm.ArmR.A.joints));
plt.A2(2, loop) = mean(diag(pandaArm.ArmR.A.min));
plt.A2(3, loop)= mean(diag(pandaArm.ArmR.A.tool));
plt.A2(4, loop) = mean(diag(pandaArm.A.con));

end