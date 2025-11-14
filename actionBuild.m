%% 定义一个空的左右臂结构体
emptyArm.names  = {};
emptyArm.pos    = [];

%% 定义整个jointStateLog的模板
emptyFrame.left  = emptyArm;
emptyFrame.right = emptyArm;

%% 预分配结构体数组
numFrames = 50;
jointStateLog = repmat(emptyFrame, numFrames, 1);

%% 填充数据
jointLimits = [-pi/2, pi/2; -pi/4, pi/4; -pi/4, pi/4; -pi, pi; -pi/2, pi/2; -pi, pi];
for i = 1:numFrames
    % 左臂
    jointStateLog(i).left.names  = {'joint1','joint2','joint3','joint4','joint5','joint6','gripper'};
    jointStateLog(i).left.pos    = [randInRange(jointLimits(1,:)), ...
                                    randInRange(jointLimits(2,:)), ...
                                    randInRange(jointLimits(3,:)), ...
                                    randInRange(jointLimits(4,:)), ...
                                    randInRange(jointLimits(5,:)), ...
                                    randInRange(jointLimits(6,:)), ...
                                    0.001 + 0.005*rand()];
    % 右臂
    jointStateLog(i).right.names = {'joint1','joint2','joint3','joint4','joint5','joint6','gripper'};
    jointStateLog(i).right.pos   = [randInRange(jointLimits(1,:)), ...
                                    randInRange(jointLimits(2,:)), ...
                                    randInRange(jointLimits(3,:)), ...
                                    randInRange(jointLimits(4,:)), ...
                                    randInRange(jointLimits(5,:)), ...
                                    randInRange(jointLimits(6,:)), ...
                                    0.001 + 0.005*rand()];
end

%% 辅助函数
function val = randInRange(range)
    val = range(1) + (range(2)-range(1))*rand();
end

%% 调用仿真
simulateDualPiper_fromJointStates(jointStateLog);
