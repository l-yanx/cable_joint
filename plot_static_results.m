function plot_static_results(time, staticResult)
%PLOT_STATIC_RESULTS 新开页面绘制静力学分析结果。

    arguments
        time (1, :) double {mustBeFinite}
        staticResult struct
    end

    forceTrajectory = staticResult.forceTrajectory;
    residualNorm = vecnorm(staticResult.residualTrajectory, 2, 1);
    minCableTension = staticResult.minCableTension;
    isFeasible = staticResult.isFeasible;

    figure(Name="绳驱关节静力学结果", Color="white", ...
        Position=[150, 150, 1100, 760]);
    layout = tiledlayout(2, 2, TileSpacing="compact", Padding="compact");
    title(layout, "绳驱关节静力学分析");

    nexttile;
    hold on;
    plot(time, forceTrajectory(1, :), LineWidth=1.2);
    plot(time, forceTrajectory(2, :), LineWidth=1.2);
    plot(time, forceTrajectory(3, :), LineWidth=1.2);
    hold off;
    grid on;
    xlabel("时间 / s");
    ylabel("张力 / N");
    title("三根绳张力");
    legend("T_1", "T_2", "T_3", Location="best");

    nexttile;
    plot(time, forceTrajectory(4, :), LineWidth=1.2);
    grid on;
    xlabel("时间 / s");
    ylabel("执行器力 / N");
    title("中央直线执行器力");
    legend("F_a", Location="best");

    nexttile;
    semilogy(time, max(residualNorm, eps), LineWidth=1.2);
    grid on;
    xlabel("时间 / s");
    ylabel("||A_q T - Q||");
    title("静力学平衡残差");

    nexttile;
    yyaxis left;
    plot(time, minCableTension, LineWidth=1.2);
    ylabel("最小绳张力 / N");

    yyaxis right;
    stairs(time, double(isFeasible), LineWidth=1.2);
    ylim([-0.1, 1.1]);
    ylabel("可行性");

    grid on;
    xlabel("时间 / s");
    title("最小绳张力与可行性");
    legend("min(T_1,T_2,T_3)", "feasible", Location="best");

    drawnow;
end
