%% ====== MATLAB 脚本：读取日志并仿真 ======
clc; clear; close all;

logFile = "F:\project_challengeCupDoubleArm\PiPERsimulation\sem_run_20250927_115922.log"; % 日志文件路径
fid = fopen(logFile,'r');
if fid < 0
    error('无法打开日志文件');
end

jointStateLog = []; % 初始化结构体数组
lineIdx = 0;

while ~feof(fid)
    tline = fgetl(fid);
    if isempty(tline) || ~contains(tline,'PublishedJointStates')
        continue;
    end
    
    % 提取 JSON 部分
    jsonStart = strfind(tline,'{');
    if isempty(jsonStart)
        continue;
    end
    jsonStr = tline(jsonStart:end);
    
    % 解析 JSON
    try
        data = jsondecode(jsonStr);
    catch
        warning('解析 JSON 出错，跳过该行');
        continue;
    end
    
    lineIdx = lineIdx + 1;
    
    % 构建 jointStateLog 结构体
    jointStateLog(lineIdx).left.names  = data.left.names;
    jointStateLog(lineIdx).left.pos    = data.left.pos;
    jointStateLog(lineIdx).left.vel    = data.left.vel;
    jointStateLog(lineIdx).left.effort = data.left.effort;
    
    jointStateLog(lineIdx).right.names  = data.right.names;
    jointStateLog(lineIdx).right.pos    = data.right.pos;
    jointStateLog(lineIdx).right.vel    = data.right.vel;
    jointStateLog(lineIdx).right.effort = data.right.effort;
    
    jointStateLog(lineIdx).timestamp = data.timestamp;
end

fclose(fid);

fprintf('共解析到 %d 帧 joint states\n', lineIdx);

%% ====== 调用仿真函数 ======
simulateDualPiper_fromJointStates(jointStateLog);
