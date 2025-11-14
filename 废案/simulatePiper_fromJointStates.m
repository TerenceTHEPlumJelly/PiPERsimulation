function simulatePiper_fromJointStates(jointStateLog)
    % jointStateLog: 结构体数组，每个元素包含:
    %   .names   -> 单元格数组 {"joint1",...,"joint6","gripper"}
    %   .pos     -> 位置向量 [7x1]
    %   .vel     -> 速度 (可选)
    %   .effort  -> 力矩 (可选)
    %   .timestamp -> 时间戳 (可选)

    % ------------------------------
    % DH参数 (示例，米)
    DH = [0   0.1   0.05   pi/2;   % Joint1
          0   0     0.20   0;      % Joint2
          0   0     0.20   0;      % Joint3
          0   0.05  0      pi/2;   % Joint4
          0   0     0      -pi/2;  % Joint5
          0   0.05  0      0];     % Joint6

    N = numel(jointStateLog);

    figure;
    axis equal;
    grid on;
    view(135,30);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([0 0.6]);
    hold on;

    % --------- 创建绘图对象 ----------
    armLinks = gobjects(6,1);
    for j=1:6
        armLinks(j) = plot3(0,0,0,'-o','LineWidth',4);
    end

    gripLeft  = plot3(0,0,0,'r-','LineWidth',6);
    gripRight = plot3(0,0,0,'b-','LineWidth',6);

    % --------- 动画循环 ----------
    for i = 1:N
        names = jointStateLog(i).names;
        pos   = jointStateLog(i).pos;

        % --- 找出对应关节 ---
        q = zeros(1,6);
        for j=1:6
            idx = find(strcmp(names, sprintf('joint%d',j)));
            if ~isempty(idx)
                q(j) = pos(idx);
            end
        end
        gripIdx = find(strcmp(names,'gripper'));
        if ~isempty(gripIdx)
            gripOpen = pos(gripIdx); % 假设就是张开一半的量
        else
            gripOpen = 0.02; % 默认值
        end

        % --------------------------
        % 正运动学
        T = eye(4);
        jointPos = zeros(3,7);
        for j=1:6
            theta = q(j);
            A = trotz(DH(j,1)+theta) * ...
                transl(0,0,DH(j,2)) * ...
                transl(DH(j,3),0,0) * ...
                trotx(DH(j,4));
            T = T * A;
            jointPos(:,j+1) = T(1:3,4);

            set(armLinks(j), 'XData', jointPos(1,j:j+1), ...
                             'YData', jointPos(2,j:j+1), ...
                             'ZData', jointPos(3,j:j+1));
        end

        % --------------------------
        % 夹爪
        z_axis = T(1:3,3);
        x_axis = T(1:3,1);
        p0 = T(1:3,4);

        pL = p0 + gripOpen * x_axis;
        pR = p0 - gripOpen * x_axis;

        fingerLen = 0.08;
        pL_tip = pL + fingerLen * z_axis;
        pR_tip = pR + fingerLen * z_axis;

        set(gripLeft,  'XData',[pL(1) pL_tip(1)], ...
                       'YData',[pL(2) pL_tip(2)], ...
                       'ZData',[pL(3) pL_tip(3)]);
        set(gripRight, 'XData',[pR(1) pR_tip(1)], ...
                       'YData',[pR(2) pR_tip(2)], ...
                       'ZData',[pR(3) pR_tip(3)]);

        drawnow;
        pause(0.05);
    end
end

% ---- 工具函数 ----
function T = transl(x,y,z)
    T = [eye(3) [x;y;z]; 0 0 0 1];
end
function T = trotz(theta)
    T = [cos(theta) -sin(theta) 0 0;
         sin(theta)  cos(theta) 0 0;
         0           0          1 0;
         0           0          0 1];
end
function T = trotx(alpha)
    T = [1 0          0           0;
         0 cos(alpha) -sin(alpha) 0;
         0 sin(alpha)  cos(alpha) 0;
         0 0           0          1];
end
