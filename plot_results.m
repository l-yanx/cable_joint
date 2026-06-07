function plot_results(time, qTrajectory, cableLength, cableSpeed, ...
        actuatorCommand, jacobianCondition)
%PLOT_RESULTS 在同一页面绘制运动学计算结果。

    figure(Name="绳驱关节运动学结果", Color="white", ...
        Position=[100, 100, 1200, 800]);
    layout = tiledlayout(2, 3, TileSpacing="compact", Padding="compact");
    title(layout, "绳驱关节运动学仿真");

    nexttile;
    plot3(rad2deg(qTrajectory(1, :)), ...
        rad2deg(qTrajectory(2, :)), qTrajectory(3, :), LineWidth=1.5);
    grid on;
    xlabel("\alpha / deg");
    ylabel("\beta / deg");
    zlabel("h / mm");
    title("三维姿态轨迹");
    view(3);

    nexttile;
    hold on;
    plot(time, cableLength(1, :), LineWidth=1.2);
    plot(time, cableLength(2, :), LineWidth=1.2);
    plot(time, cableLength(3, :), LineWidth=1.2);
    hold off;
    grid on;
    xlabel("时间 / s");
    ylabel("绳长 / mm");
    title("三根绳长轨迹");
    legend("l_1", "l_2", "l_3", Location="best");

    nexttile;
    hold on;
    plot(time, cableSpeed(1, :), LineWidth=1.2);
    plot(time, cableSpeed(2, :), LineWidth=1.2);
    plot(time, cableSpeed(3, :), LineWidth=1.2);
    hold off;
    grid on;
    xlabel("时间 / s");
    ylabel("绳速 / (mm/s)");
    title("三根绳速轨迹");
    legend("dl_1/dt", "dl_2/dt", "dl_3/dt", Location="best");

    actuatorAngleDeg = rad2deg(actuatorCommand);
    nexttile;
    yyaxis left;
    hold on;
    plot(time, actuatorAngleDeg(1, :), LineWidth=1.2);
    plot(time, actuatorAngleDeg(2, :), LineWidth=1.2);
    plot(time, actuatorAngleDeg(3, :), LineWidth=1.2);
    hold off;
    ylabel("卷筒角度 / deg");

    yyaxis right;
    plot(time, qTrajectory(3, :), LineWidth=1.2);
    ylabel("位移 / mm");

    grid on;
    xlabel("时间 / s");
    title("执行器执行参数");
    legend("\theta_1", "\theta_2", "\theta_3", "h", Location="best");

    nexttile;
    semilogy(time, jacobianCondition, LineWidth=1.2);
    grid on;
    xlabel("时间 / s");
    ylabel("雅各比条件数 cond(J)");
    title("雅各比矩阵条件数");

    drawnow;
end
