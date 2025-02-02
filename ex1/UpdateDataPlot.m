function [ plt ] = UpdateDataPlot( plt, uvms, t, loop )

    % this function samples the variables contained in the structure uvms
    % and saves them in arrays inside the struct plt
    % this allows to have the time history of the data for later plots
    
    % you can add whatever sampling you need to do additional plots
    % plots are done in the PrintPlot.m script
    
    plt.t(loop) = t;
    
    plt.toolPos(:, loop) = uvms.wTt(1:3,4);
    
    plt.q(:, loop) = uvms.q;
    plt.q_dot(:, loop) = uvms.q_dot;
    
    plt.p(:, loop) = uvms.p;
    plt.p_dot(:, loop) = uvms.p_dot;
    
    %plt.xdot_jl(:, loop) = uvms.xdot.jl;
    %plt.xdot_mu(:, loop) = uvms.xdot.mu;
    plt.xdot_t(:, loop) =  blkdiag(uvms.wTv(1:3,1:3), uvms.wTv(1:3,1:3))*uvms.xdot.tool;
    
    % plt.a(1:7, loop) = diag(uvms.A.jl);
    plt.A(1, loop) = uvms.A.ha;
    plt.A(2, loop) = uvms.A.ma;
    plt.A(3, loop)= mean(diag(uvms.A.v));
    plt.A(4, loop) = mean(diag(uvms.A.landing));
    plt.A(5, loop) = mean(diag(uvms.A.tool));
    plt.A(6, loop) = mean(diag(uvms.A.closer));
    
    plt.toolx(loop) = uvms.wTt(1,4);
    plt.tooly(loop) = uvms.wTt(2,4);
    end
    