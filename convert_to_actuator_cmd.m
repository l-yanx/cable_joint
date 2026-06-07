function actuatorCommand = convert_to_actuator_cmd(cableLength, drumRadius)
%CONVERT_TO_ACTUATOR_CMD 将绳长变化量转换为卷筒转角。

    actuatorCommand = (cableLength - cableLength(:, 1)) / drumRadius;
end
