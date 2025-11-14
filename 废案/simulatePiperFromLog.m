function simulatePiperFromLog(logFile, DH_left, DH_right, dt, options)
    arguments
        logFile char                      % 日志文件路径（JSON 格式）
        DH_left double
        DH_right double
        dt double = 0.1
        options.showFrames logical = true
        options.trail logical = false
    end

    % === 读取 JSON ===
    fid = fopen(logFile,'r');
    raw = fread(fid,inf);
    str = char(raw');
    fclose(fid);
    data = jsondecode(str);

    % === 提取关节轨迹 ===
    jointNames = data.left.names;
    numJoints = numel(jointNames);

    jointTrajLeft  = data.left.pos;   % 单次输入 -> 转成一行
    jointTrajRight = data.right.pos;

    % 如果输入里有时间序列（多个 pos ），则拼接成矩阵
    if iscell(jointTrajLeft)
        jointTrajLeft  = cell2mat(jointTrajLeft(:));
        jointTrajRight = cell2mat(jointTrajRight(:));
    else
        jointTrajLeft  = jointTrajLeft(:)';   % 转成 1×7
        jointTrajRight = jointTrajRight(:)';  % 转成 1×7
    end

    % 去掉 gripper，只保留 6 个旋转关节
    jointTrajLeft  = jointTrajLeft(:,1:6);
    jointTrajRight = jointTrajRight(:,1:6);

    % === 开始绘图 ===
    figure; hold on; grid on;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    axis equal;
    view(135,30);
    xlim([-1.5 1.5]); ylim([-1.5 1.5]); zlim([0 1.5]);

    % === 基座绘制 ===
    [Xc,Yc,Zc] = cylinder(0.08,30);
    Zc = Zc * 0.05;
    surf(Xc-0.3, Yc, Zc, 'FaceColor',[0.5 0.5 0.5],'EdgeColor','none'); % 左基座
    surf(Xc+0.3, Yc, Zc, 'FaceColor',[0.5 0.5 0.5],'EdgeColor','none'); % 右基座

    hLeft = plot3(0,0,0,'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor','b');
    hRight = plot3(0,0,0,'-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor','r');

    if options.trail
        trailLeft = animatedline('Color','b','LineWidth',1);
        trailRight = animatedline('Color','r','LineWidth',1);
    end

    % === 动画循环 ===
    for k=1:size(jointTrajLeft,1)
        qL = jointTrajLeft(k,:);
        qR = jointTrajRight(k,:);

        % 左臂前向运动学
        T = eye(4);
        ptsL = [0 0 0];
        for i=1:size(DH_left,1)
            T = T * dh2mat(DH_left(i,:), qL(i));
            ptsL = [ptsL; T(1:3,4)'];
        end

        % 右臂前向运动学（基座偏移）
        T = eye(4); T(1,4) = 0.6;
        ptsR = [0.6 0 0];
        for i=1:size(DH_right,1)
            T = T * dh2mat(DH_right(i,:), qR(i));
            ptsR = [ptsR; T(1:3,4)'];
        end

        % 更新绘制
        set(hLeft, 'XData', ptsL(:,1)-0.3, 'YData', ptsL(:,2), 'ZData', ptsL(:,3));
        set(hRight,'XData', ptsR(:,1)-0.3, 'YData', ptsR(:,2), 'ZData', ptsR(:,3));

        if options.trail
            addpoints(trailLeft, ptsL(end,1)-0.3, ptsL(end,2), ptsL(end,3));
            addpoints(trailRight,ptsR(end,1)-0.3, ptsR(end,2), ptsR(end,3));
        end

        drawnow;
        pause(dt);
    end
end

function T = dh2mat(dhRow, q)
    theta = dhRow(1)+q;
    d     = dhRow(2);
    a     = dhRow(3);
    alpha = dhRow(4);

    T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
         sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
         0           sin(alpha)             cos(alpha)            d;
         0           0                      0                     1];
end
