function figureHandle = plot_workspace_cloud(result)
%PLOT_WORKSPACE_CLOUD Plot feasible alpha-beta-height workspace samples.

    requiredFields = ["pose", "minCableTension", "isFeasible"];
    if ~isstruct(result) || ~all(isfield(result, requiredFields))
        error("CableJointWorkspace:InvalidConfiguration", ...
            "绘图结果缺少必要字段。");
    end
    sampleCount = size(result.pose, 2);
    if size(result.pose, 1) ~= 3 ...
            || ~isequal(size(result.minCableTension), [1, sampleCount]) ...
            || ~isequal(size(result.isFeasible), [1, sampleCount])
        error("CableJointWorkspace:InvalidConfiguration", ...
            "绘图结果数组尺寸不一致。");
    end

    figureHandle = figure(Name="绳驱关节正运动学工作空间", ...
        Color="w", Position=[100, 100, 1000, 750]);
    axesHandle = axes(figureHandle);
    hold(axesHandle, "on");
    grid(axesHandle, "on");
    box(axesHandle, "on");
    view(axesHandle, [-37.5, 30]);

    feasible = logical(result.isFeasible);
    if any(feasible)
        scatter3(axesHandle, rad2deg(result.pose(1, feasible)), ...
            rad2deg(result.pose(2, feasible)), result.pose(3, feasible), ...
            12, result.minCableTension(feasible), "filled");
        colorbarHandle = colorbar(axesHandle);
        colorbarHandle.Label.String = "最小绳张力 / N";
    else
        warning("CableJointWorkspace:NoFeasiblePoints", ...
            "当前分析结果中没有可行工作空间点。");
        text(axesHandle, 0.5, 0.5, 0.5, "没有可行工作空间点", ...
            Units="normalized", HorizontalAlignment="center", FontSize=12);
    end
    xlabel(axesHandle, "\alpha / deg");
    ylabel(axesHandle, "\beta / deg");
    zlabel(axesHandle, "h / mm");
    title(axesHandle, "正运动学可行工作空间");
end
