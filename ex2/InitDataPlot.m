function [plt] = InitDataPlot( maxloops)
    plt.t = zeros(1, maxloops);
    
    plt.q = zeros(7, maxloops);
    plt.q_dot = zeros(7, maxloops);
    plt.q2 = zeros(7, maxloops);
    plt.q_dot2 = zeros(7, maxloops);


    plt.A = zeros(4, maxloops);
    plt.A2 = zeros(4, maxloops);


    %End effector velocities (Left Arm)
    plt.xdot = zeros(6, maxloops);
    plt.qe_dot = zeros(6, maxloops);

    %End effector velocities (Right Arm)
    plt.xdot2 = zeros(6, maxloops);
    plt.qe_dot2 = zeros(6, maxloops);

end

