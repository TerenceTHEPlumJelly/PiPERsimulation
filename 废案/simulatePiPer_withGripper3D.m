function simulatePiPer_withGripper3D(jointTraj, gripTraj)
    % jointTraj: N x 6 关节角度轨迹 (弧度制)
    % gripTraj:  N x 1 抓夹开合量 (夹爪张开距离的一半，单位 m)

    % ------------------------------
    % DH参数 (示例，单位: 米)
    DH = [0   0.1   0.05   pi/2;   % Joint1
          0   0     0.20   0;      % Joint2
          0   0     0.20   0;      % Joint3
          0   0.05  0      pi/2;   % Joint4
          0   0     0      -pi/2;  % Joint5
          0   0.05  0      0];     % Joint6

    % 仿真步数
    N = size(jointTraj,1);

    figure;
    axis equal;
    grid on;
    view(135,30);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([0 0.6]);
    hold on;

    % --------- 创建绘图对象 ----------
    % 机械臂连杆
    armLinks = gobjects(6,1);
    for j=1:6
        armLinks(j) = plot3(0,0,0,'-o','LineWidth',4);
    end

    % 夹爪（两根小杆子）
    gripLeft  = plot3(0,0,0,'r-','LineWidth',6);
    gripRight = plot3(0,0,0,'b-','LineWidth',6);

    % --------- 仿真循环 ----------
    for i = 1:N
        % --------------------------
        % 计算机械臂正运动学
        T = eye(4);
        jointPos = zeros(3, size(DH,1)+1);
        jointPos(:,1) = [0;0;0];
        for j = 1:6
            theta = jointTraj(i,j);
            A = trotz(DH(j,1)+theta) * ...
                transl(0,0,DH(j,2)) * ...
                transl(DH(j,3),0,0) * ...
                trotx(DH(j,4));
            T = T * A;
            jointPos(:,j+1) = T(1:3,4);

            % 更新连杆
            set(armLinks(j), 'XData', jointPos(1,j:j+1), ...
                             'YData', jointPos(2,j:j+1), ...
                             'ZData', jointPos(3,j:j+1));
        end

        % --------------------------
        % 计算末端夹爪位置
        gripOpen = gripTraj(i); % 张开量的一半
        T_end = T;
        z_axis = T_end(1:3,3); % 前伸方向
        x_axis = T_end(1:3,1); % 左右方向

        % 根部
        p0 = T_end(1:3,4);

        % 左右指根
        pL = p0 + gripOpen * x_axis;
        pR = p0 - gripOpen * x_axis;

        % 指尖
        fingerLen = 0.08;
        pL_tip = pL + fingerLen * z_axis;
        pR_tip = pR + fingerLen * z_axis;

        % 更新夹爪
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
