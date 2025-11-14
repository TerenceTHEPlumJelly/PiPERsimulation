% PiPER_RRRRRR_simulation.m
% MATLAB simulation for a 6-DOF RRRRRR manipulator (松灵 PiPER 风格)
% 
% 主要接口:
%   result = simulatePiperRRRRRR(DH, jointTraj, dt, opts)
%
% 输入:
%   DH        - 6x4 Denavit-Hartenberg 参数，每行 [alpha, a, d, theta_offset]
%               alpha (rad), a (m), d (m), theta_offset (rad). 对于纯转动关节，d为常量，
%               关节角度由 jointTraj 给出并加上 theta_offset.
%   jointTraj - N x 6 矩阵，N帧，每帧为6个关节角（弧度）
%   dt        - 帧间时间（秒）
%   opts      - 结构体，可选字段:
%               .showFrames (logical) - 是否在每个关节处显示坐标系，默认 false
%               .trail (logical)      - 是否绘制末端轨迹，默认 true
%               .linkWidth (double)   - 连接线宽度，默认 3
%               .axisLimit (1x6)      - [xmin xmax ymin ymax zmin zmax]，默认自动
%
% 输出:
%   result - 结构体，包含
%       .jointPos (N x 6 x 3) - 每帧各关节在基坐标系下的位置
%       .EEpose   (N x 4 x 4)  - 每帧末端位姿齐次变换矩阵
%       .time    (N x 1)       - 时间戳
%
% 使用说明示例（文件末尾有示例）：
%   % 定义一个简单的 DH（示例参数，仅用于演示）
%   DH = [0    0.05  0.10  0;    % alpha a d theta_offset
%         -pi/2 0.0   0.0   0;
%         0     0.25  0.0   0;
%         -pi/2 0.0   0.18  0;
%         pi/2  0.0   0.0   0;
%         -pi/2 0.0   0.06  0];
%   % 生成一个简单的 jointTraj：N帧，每关节做sine运动
%   t = 0:0.05:4;
%   jointTraj = zeros(length(t),6);
%   for i=1:6
%       jointTraj(:,i) = 0.6*sin(2*pi*(0.2+i*0.02)*t) + 0.1*i;
%   end
%   simulatePiperRRRRRR(DH, jointTraj, 0.05);
%
% ---------- 主函数 ----------
function result = simulatePiperRRRRRR(DH, jointTraj, dt, opts)
    narginchk(3,4);
    if nargin<4 || isempty(opts), opts = struct(); end
    if ~isfield(opts,'showFrames'), opts.showFrames = false; end
    if ~isfield(opts,'trail'), opts.trail = true; end
    if ~isfield(opts,'linkWidth'), opts.linkWidth = 3; end

    % 验证输入
    [nFrames, nJ] = size(jointTraj);
    if nJ~=6, error('jointTraj must be N x 6'); end
    if size(DH,1)~=6 || size(DH,2)~=4, error('DH must be 6x4'); end

    % 预分配
    jointPos = zeros(nFrames,6,3);
    EEpose = zeros(nFrames,4,4);
    time = ((0:nFrames-1)'*dt);

    % 初始化图形
    fig = figure('Name','PiPER RRRRRR Simulation','NumberTitle','off');
    ax = axes('Parent',fig); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(3);
    title(ax,'PiPER-style RRRRRR Manipulator Simulation');
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');

    % 初次计算以设置轴范围
    [p0, T0] = compute_frame_positions(DH, jointTraj(1,:));
    allPts = reshape(p0,[],3);
    if isfield(opts,'axisLimit') && ~isempty(opts.axisLimit)
        axis(ax, opts.axisLimit);
    else
        pad = 0.2; mins = min(allPts)-pad; maxs = max(allPts)+pad;
        axis(ax,[mins(1) maxs(1) mins(2) maxs(2) mins(3) maxs(3)]);
    end

    % 绘制初始连杆
    linkLines = gobjects(6,1);
    jointMarkers = gobjects(6,1);
    for j=1:6
        pA = p0(j,:);
        if j==1
            pB = [0 0 0];
        else
            pB = p0(j-1,:);
        end
        % 线段从上一关节到当前关节（为了可视化，从基到末端）
        if j==1
            x = [0, p0(1,1)]; y=[0,p0(1,2)]; z=[0,p0(1,3)];
        else
            x = [p0(j-1,1), p0(j,1)]; y=[p0(j-1,2), p0(j,2)]; z=[p0(j-1,3), p0(j,3)];
        end
        linkLines(j) = plot3(ax,x,y,z,'LineWidth',opts.linkWidth);
        jointMarkers(j) = plot3(ax,p0(j,1),p0(j,2),p0(j,3),'o','MarkerSize',6,'MarkerFaceColor',[0 0.4470 0.7410]);
    end
    eeTrail = plot3(ax, T0(1,4), T0(2,4), T0(3,4), '.','MarkerSize',8);

    % 可选：显示每节坐标系
    frameAxes = gobjects(6,1);
    if opts.showFrames
        for j=1:6
            frameAxes(j) = draw_frame(ax, T0(:,:,j), 0.04);
        end
    end

    % 动画循环
    for k=1:nFrames
        q = jointTraj(k,:);
        [p, T_all] = compute_frame_positions(DH, q);
        jointPos(k,:,:) = p;
        EEpose(k,:,:) = T_all(:,:,end);

        % 更新线条和关节点
        for j=1:6
            if j==1
                xs = [0 p(1,1)]; ys=[0 p(1,2)]; zs=[0 p(1,3)];
            else
                xs = [p(j-1,1) p(j,1)]; ys=[p(j-1,2) p(j,2)]; zs=[p(j-1,3) p(j,3)];
            end
            set(linkLines(j),'XData',xs,'YData',ys,'ZData',zs);
            set(jointMarkers(j),'XData',p(j,1),'YData',p(j,2),'ZData',p(j,3));
        end

        % 更新末端轨迹
        eeX = squeeze(EEpose(1:k,1,4)); eeY = squeeze(EEpose(1:k,2,4)); eeZ = squeeze(EEpose(1:k,3,4));
        if opts.trail
            set(eeTrail,'XData',eeX,'YData',eeY,'ZData',eeZ,'Marker','.','MarkerSize',6,'LineStyle','none');
        else
            set(eeTrail,'XData',T_all(1,4),'YData',T_all(2,4),'ZData',T_all(3,4));
        end

        % 更新坐标系（如果选中）
        if opts.showFrames
            for j=1:6
                update_frame(frameAxes(j), T_all(:,:,j));
            end
        end

        drawnow;
        pause(dt);
    end

    % 返回结果
    result.jointPos = jointPos;
    result.EEpose = EEpose;
    result.time = time;
end

% ---------- 辅助函数：计算每个关节在基坐标系下的位置与每节的变换矩阵 ----------
function [positions, T_all] = compute_frame_positions(DH, q)
    % DH: 6x4 [alpha a d theta_offset]
    % q: 1x6 实际关节角（弧度）
    T = eye(4);
    T_all = zeros(4,4,6);
    positions = zeros(6,3);
    for i=1:6
        alpha = DH(i,1);
        a     = DH(i,2);
        d     = DH(i,3);
        theta_offset = DH(i,4);
        theta = q(i) + theta_offset;
        A = dh_transform(alpha,a,d,theta);
        T = T * A;
        T_all(:,:,i) = T;
        positions(i,:) = T(1:3,4)';
    end
end

% ---------- DH 变换矩阵（标准 DH） ----------
function A = dh_transform(alpha,a,d,theta)
    ca = cos(alpha); sa = sin(alpha);
    ct = cos(theta); st = sin(theta);
    A = [ ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1];
end

% ---------- 绘制坐标系（用三条短线表示） ----------
function h = draw_frame(ax, T, scale)
    % T: 4x4
    origin = T(1:3,4)';
    xdir = T(1:3,1)'; ydir = T(1:3,2)'; zdir = T(1:3,3)';
    hx = plot3(ax, [origin(1) origin(1)+scale*xdir(1)], [origin(2) origin(2)+scale*xdir(2)], [origin(3) origin(3)+scale*xdir(3)], '-','LineWidth',2);
    hy = plot3(ax, [origin(1) origin(1)+scale*ydir(1)], [origin(2) origin(2)+scale*ydir(2)], [origin(3) origin(3)+scale*ydir(3)], '-','LineWidth',2);
    hz = plot3(ax, [origin(1) origin(1)+scale*zdir(1)], [origin(2) origin(2)+scale*zdir(2)], [origin(3) origin(3)+scale*zdir(3)], '-','LineWidth',2);
    h = [hx,hy,hz];
end

function update_frame(h, T)
    origin = T(1:3,4)';
    xdir = T(1:3,1)'; ydir = T(1:3,2)'; zdir = T(1:3,3)';
    scale = norm([h(1).XData(2)-h(1).XData(1), h(1).YData(2)-h(1).YData(1), h(1).ZData(2)-h(1).ZData(1)]);
    set(h(1),'XData',[origin(1) origin(1)+scale*xdir(1)],'YData',[origin(2) origin(2)+scale*xdir(2)],'ZData',[origin(3) origin(3)+scale*xdir(3)]);
    set(h(2),'XData',[origin(1) origin(1)+scale*ydir(1)],'YData',[origin(2) origin(2)+scale*ydir(2)],'ZData',[origin(3) origin(3)+scale*ydir(3)]);
    set(h(3),'XData',[origin(1) origin(1)+scale*zdir(1)],'YData',[origin(2) origin(2)+scale*zdir(2)],'ZData',[origin(3) origin(3)+scale*zdir(3)]);
end

% ---------- 文件末尾示例（取消注释并运行以查看演示） ----------
% % 示例：
% % DH示例（请用真实PiPER参数替换）
% DH = [0    0.05  0.10  0;
%       -pi/2 0.0   0.0   0;
%       0     0.25  0.0   0;
%       -pi/2 0.0   0.18  0;
%       pi/2  0.0   0.0   0;
%       -pi/2 0.0   0.06  0];
% t = 0:0.05:6;
% jointTraj = zeros(length(t),6);
% for i=1:6
%     jointTraj(:,i) = 0.4*sin(2*pi*(0.1+0.05*i)*t) + 0.2*(i-3);
% end
% simulatePiperRRRRRR(DH, jointTraj, 0.05, struct('showFrames',true,'trail',true));
