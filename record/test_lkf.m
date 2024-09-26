clear variables; close all
dataRec=load('datalog.log');
simTime=dataRec(:,1:1);

% %
mujoco_pos=dataRec(:,2:4);
mujoco_vel=dataRec(:,5:7);
pos=dataRec(:,8:10);
vel=dataRec(:,11:13);

[rowt,colt] = size(simTime);
ranget = 1:rowt;

figure("Name","base pos&vel", 'Position', [100, 100, 2000, 1200]);
for i = 1:3
    subplot(2,3,i);
    plot(simTime(ranget,1), mujoco_pos(ranget,i) ,'b-',...
         simTime(ranget,1), pos(ranget,i) ,'r--', 'LineWidth', 2)
    legend('mujoco pos', 'true pos'); % 添加图例
    title(['Position ', char('X'+i-1)]); % 添加标题：位置的 xyz
end
for i =1:3
    subplot(2,3,i+3);
    plot(simTime(ranget,1), mujoco_vel(ranget,i) ,'b-',...
         simTime(ranget,1), vel(ranget,i) ,'r--', 'LineWidth', 2)
    legend('mujoco vel', 'true vel'); % 添加图例
    title(['Velocity ', char('X'+i-1)]); % 添加标题：速度的 xyz
end
