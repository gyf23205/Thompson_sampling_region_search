clc;
clear;
maindir = 'G:\matlab\workspace\results';
subdir  = dir( maindir );
hold on;
reward_history=zeros(5,400);
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
                reward_history(i-2,time)=reward_history(i-2,time)+1;
            end
        end
    end
%     for j=2:400
%         reward_history(i-2,j)=reward_history(i-2,j-1)+reward_history(i-2,j);
%     end  
end
% plot(reward_history(2,:)-reward_history(1,:),'LineWidth',2)
% plot(reward_history(3,:)-reward_history(2,:),'LineWidth',2)
% plot(reward_history(4,:)-reward_history(3,:),'LineWidth',2)
% plot(reward_history(5,:)-reward_history(4,:),'LineWidth',2)
count=1;
avg1=[];
sum=0;
for i=1:length(reward_history(1,:))
    sum=sum+reward_history(1,i);
    count=count+1;
    if(count==10)
        avg1=[avg1,sum/10];
        sum=0;
        count=0;
    end
end
count=1;
avg2=[];
sum=0;
for i=1:length(reward_history(2,:))
    sum=sum+reward_history(2,i);
    count=count+1;
    if(count==10)
        avg2=[avg2,sum/10];
        sum=0;
        count=0;
    end
end
plot(avg1,'Marker','*')
plot(avg2,'Marker','s')
title('reward gain vs iteartion')
xlabel('iteration')
ylabel('reward')
legend({'rob num=20','rob num=30'},'Location','northwest')