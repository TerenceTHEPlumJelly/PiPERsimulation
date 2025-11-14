function simulateDualPiper_fromJointStates(jointStateLog)
    % DH参数
    DH = [0   0.1   0.05   pi/2;
          0   0     0.20   0;
          0   0     0.20   0;
          0   0.05  0      pi/2;
          0   0     0      -pi/2;
          0   0.05  0      0];

    N = numel(jointStateLog);

    % 左右臂基座偏移
    baseOffsetL = [0, -0.3, 0.05];  % 左臂在 y 负方向
    baseOffsetR = [0,  0.3, 0.05];  % 右臂在 y 正方向

    %% 预计算所有关节位置，用于固定坐标轴
    allPos = [];
    for i = 1:N
        [qL, ~] = extractJointStates(jointStateLog(i).left);
        jointPosL = forwardKinematics(qL, DH, baseOffsetL);
        allPos = [allPos, jointPosL];

        [qR, ~] = extractJointStates(jointStateLog(i).right);
        jointPosR = forwardKinematics(qR, DH, baseOffsetR);
        allPos = [allPos, jointPosR];
    end

    margin = 0.3;  % 加大一些边距，保证夹爪完整显示
    xRange = [min(allPos(1,:))-margin, max(allPos(1,:))+margin];
    yRange = [min(allPos(2,:))-margin, max(allPos(2,:))+margin];
    zRange = [0, max(allPos(3,:))+margin];

    %% 绘图初始化
    figure;
    axis equal; grid on; view(135,30);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    hold on;
    xlim(xRange); ylim(yRange); zlim(zRange); % 固定坐标轴

    % 底座
    baseRadius = 0.1; baseHeight = 0.05;
    [Xc,Yc,Zc] = cylinder(baseRadius,50); Zc = Zc*baseHeight;

    % 左臂底座
    surf(Xc + baseOffsetL(1), Yc + baseOffsetL(2), Zc + baseOffsetL(3), ...
         'FaceColor',[0.6 0.6 0.6],'EdgeColor','none');

    % 右臂底座
    surf(Xc + baseOffsetR(1), Yc + baseOffsetR(2), Zc + baseOffsetR(3), ...
         'FaceColor',[0.6 0.6 0.6],'EdgeColor','none');

    % 左臂
    armLinksL = gobjects(6,1);
    for j=1:6, armLinksL(j) = plot3(0,0,0,'-o','LineWidth',3,'Color',[0 0 1]); end
    gripLeftL  = plot3(0,0,0,'r-','LineWidth',6);
    gripRightL = plot3(0,0,0,'b-','LineWidth',6);

    % 右臂
    armLinksR = gobjects(6,1);
    for j=1:6, armLinksR(j) = plot3(0,0,0,'-o','LineWidth',3,'Color',[0.2 0.7 0.2]); end
    gripLeftR  = plot3(0,0,0,'m-','LineWidth',6);
    gripRightR = plot3(0,0,0,'c-','LineWidth',6);

    %% 动画循环
    for i = 1:N
        % 左臂
        [qL, gripL] = extractJointStates(jointStateLog(i).left);
        updateArm(qL, gripL, DH, armLinksL, gripLeftL, gripRightL, baseOffsetL);

        % 右臂
        [qR, gripR] = extractJointStates(jointStateLog(i).right);
        updateArm(qR, gripR, DH, armLinksR, gripLeftR, gripRightR, baseOffsetR);

        drawnow; pause(0.05);
    end
end

%% ===== 子函数 =====
function [q, grip] = extractJointStates(data)
    q = zeros(1,6);
    for j=1:6
        idx = find(strcmp(data.names, sprintf('joint%d',j)));
        if ~isempty(idx), q(j) = data.pos(idx); end
    end
    gripIdx = find(strcmp(data.names,'gripper'));
    if ~isempty(gripIdx), grip = data.pos(gripIdx); else, grip = 0.02; end
end

function jointPos = forwardKinematics(q, DH, baseOffset)
    % 基座初始旋转：x 轴朝全局 x 负方向
    T = transl(baseOffset(1), baseOffset(2), baseOffset(3)) * trotz(pi);
    jointPos = zeros(3,7);
    jointPos(:,1) = baseOffset(:);
    for j=1:6
        theta = q(j);
        A = trotz(DH(j,1)+theta) * transl(0,0,DH(j,2)) * transl(DH(j,3),0,0) * trotx(DH(j,4));
        T = T*A;
        jointPos(:,j+1) = T(1:3,4);
    end
end

function updateArm(q, gripOpen, DH, armLinks, gripLeft, gripRight, baseOffset)
    T = transl(baseOffset(1),baseOffset(2),baseOffset(3)) * trotz(pi); % 保证 x 轴朝负
    jointPos = zeros(3,7);
    jointPos(:,1) = baseOffset(:);

    for j=1:6
        theta = q(j);
        A = trotz(DH(j,1)+theta) * transl(0,0,DH(j,2)) * transl(DH(j,3),0,0) * trotx(DH(j,4));
        T = T*A;
        jointPos(:,j+1) = T(1:3,4);

        set(armLinks(j),'XData',jointPos(1,j:j+1),...
                        'YData',jointPos(2,j:j+1),...
                        'ZData',jointPos(3,j:j+1));
    end

    % 夹爪
    z_axis = T(1:3,3); x_axis = T(1:3,1); p0 = T(1:3,4);
    pL = p0 + gripOpen * x_axis;
    pR = p0 - gripOpen * x_axis;
    fingerLen = 0.08;
    pL_tip = pL + fingerLen * z_axis;
    pR_tip = pR + fingerLen * z_axis;

    set(gripLeft, 'XData',[pL(1) pL_tip(1)],...
                   'YData',[pL(2) pL_tip(2)],...
                   'ZData',[pL(3) pL_tip(3)]);
    set(gripRight,'XData',[pR(1) pR_tip(1)],...
                   'YData',[pR(2) pR_tip(2)],...
                   'ZData',[pR(3) pR_tip(3)]);
end

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
