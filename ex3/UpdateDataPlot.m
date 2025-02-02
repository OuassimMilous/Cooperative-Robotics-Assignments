function [ plt ] = UpdateDataPlot( plt, pandaArm1, pandaArm2, t, loop, mission )


% this function samples the variables contained in the structure pandaArm
% and saves them in arrays inside the struct plt
% this allows to have the time history of the datauvms for later plots

% you can add whatever sampling you need to do additional plots
% plots are done in the PrintPlot.m script


plt.t(:, loop) = t;
plt.q(:, loop) = pandaArm1.q;
plt.q_dot(:, loop) = pandaArm1.q_dot;
plt.q2(:, loop) = pandaArm2.q;
plt.q_dot2(:, loop) = pandaArm2.q_dot;



% Plot: desired object velocity
plt.xdot(:, loop) = pandaArm1.xdot.tool;
plt.qe_dot(:, loop) = pandaArm1.wJt(1:7) * pandaArm1.q;

plt.xdot2(:, loop) = pandaArm2.xdot.tool;
plt.qe_dot2(:, loop) = pandaArm2.wJt(8:14) * pandaArm2.q;

% Plot: manipulability task activation function
plt.A(1, loop) = mean(diag(pandaArm1.A.joints));
plt.A(2, loop) = mean(diag(pandaArm1.A.min));
plt.A(3, loop)= mean(diag(pandaArm1.A.tool));
plt.A(4, loop) = mean(diag(pandaArm1.A.coop));

plt.A2(1, loop) = mean(diag(pandaArm2.A.joints));
plt.A2(2, loop) = mean(diag(pandaArm2.A.min));
plt.A2(3, loop)= mean(diag(pandaArm2.A.tool));
plt.A2(4, loop) = mean(diag(pandaArm1.A.coop));

end