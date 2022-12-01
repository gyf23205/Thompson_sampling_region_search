clc;
clear;
maindir = 'G:\matlab\workspace\results';
subdir  = dir( maindir );
robot_num=[];
avg_reward=[];
for i = 1 : length( subdir )
        if( isequal( subdir( i ).name, '.' )||...
        isequal( subdir( i ).name, '..')||...
        subdir( i ).isdir)               % 如果是目录则跳过
        continue;
        end
    filepath = fullfile( maindir, subdir( i ).name);
    load(filepath);%读取一次运行的结果
    reward = 0;
    duration = 0;
    for robot = 1:length(h.robots)
        result= h.robots(robot).data.z;
        data_size=(size(result));
        duration = data_size(2);
        for time = 1:duration
            one_time_result=result(time);
            s=size(one_time_result{1,1});
            if(s(1)>0)
                reward=reward+1;
            end
        end
    end
    robot_num=[robot_num,length(h.robots)];
    avg_reward=[avg_reward,reward/((i-2)*5*duration)];
end
% dis=[];
% for i=2:11
%     dis(end+1)=avg_reward(i)-avg_reward(i-1);
% end
% 
% plot(dis,'Color','r','Marker','s','MarkerEdgeColor','blue','MarkerFaceColor','blue')
% hold on
% plot([1,10],[0,0],'--')
% set(gca,'xtick',1:1:10,'xticklabel',{'25-26','26-27','27-28','28-29','29-30',...
%     '30-31','31-32','32-33','33-34','34-35'});
plot(robot_num,avg_reward,'b', 'LineWidth',2,'Marker','o')
title('efficiency vs robot num')
xlabel('Robot number')
ylabel('efficiency')