
clc;clear;close all;
%% 队列仿真

N = 20;          % 队列的个数
Time = 20;       % 总共的仿真时间
T_delta = 0.01;  % 仿真步长

Tao = 0.5;       % 机械时延

%% 队列个体的状态量

X = zeros(Time/T_delta,N);
V = zeros(Time/T_delta,N);
a = zeros(Time/T_delta,N);
U = zeros(Time/T_delta,N);

d = 10;   % 期望的跟车误差
%% leader 的状态
a0 = zeros(Time/T_delta,1);
v0 = zeros(Time/T_delta,1);
x0 = zeros(Time/T_delta,1);

  %% 恒定加速度
 a0(5/T_delta:10/T_delta) = 2;
 v0(1) = 20;

for i = 1:N
    X(1,i) = x0(1)-i*d - (rand(1)-1)*2;
    V(1,i) = 20;
end


%% leader 状态更新
for i = 2:Time/T_delta
    v0(i) = v0(i-1)+a0(i)*T_delta;
    x0(i) = x0(i-1)+v0(i)*T_delta;    
end



%% 队列状态更新

% 控制器的反馈系数
 k1 = 1; k2 = 2; k3 = 1;    % 稳定 只要速度有反馈，肯定会使得增益超过1
%    k1 = 1; k2 = 0.2; k3 = 1;  % 不稳定 只要速度有反馈，肯定会使得增益超过1

for i = 2:Time/T_delta
    

    %% 第j辆车的更新状态 :用follower与自己的全部状态差进行反馈
    
    for j = 1:N
        % 通信权重一致  
        % 计算全部follower的状态差
        Ex = 0;Ev = 0; Ea = 0;        
        for jj = 1:N
                Ex = Ex + X(i-1,j)-X(i-1,jj) - (jj-j)*d;
                Ev = Ev + V(i-1,j)-V(i-1,jj);
                Ea = Ea + V(i-1,j)-V(i-1,jj);
        end
   
        U(i,j) = - k1*Ex - k2*Ev-k3*Ea;
        a(i,j) = (Tao-T_delta)/Tao*a(i-1,j) + T_delta/Tao*U(i,j);
        V(i,j) = V(i-1,j)+a(i,j)*T_delta;
        X(i,j) = X(i-1,j)+V(i,j)*T_delta; 
    end
    
end

%% 绘图

close all;
t = T_delta:T_delta:Time;
ColorSet = {'r','b','k','g','m','r-.','b-.','k-.','g-.','m-.'};

h = zeros(N,1);
figure;
h(1) = plot(t,t-t,'r','linewidth',2);hold on;

% 跟车误差
for i = 1:N-1
   h(i+1) = plot(t,X(:,i)-X(:,i+1)-d,ColorSet{mod(i+1,10)+1},'linewidth',2);box off;xlabel('Time (s)');hold on
end
set(gcf,'Position',[250 150 300 350]);
box off, grid on;
h1 = legend([h(1) h(3) h(5) h(7) h(10)],'1','3','5','7','10','location','NorthEast');ylabel('Spacing Error (m)')
set(h1,'box','off')

figure;
h = zeros(N,1);
% 速度
for i = 1:10
   h(i) = plot(t,V(:,i),ColorSet{mod(i+1,10)+1},'linewidth',2);box off;xlabel('Time (s)');hold on
end
set(gcf,'Position',[250 150 300 350]);
box off, grid on;
h2 = legend([h(1) h(3) h(5) h(7) h(10)],'1','3','5','7','10','location','NorthEast');
set(h2,'box','off');ylabel('Speed')

figure;
h = zeros(N,1);
% 加速度
for i = 1:10
   h(i) = plot(t,a(:,i),ColorSet{mod(i+1,10)+1},'linewidth',2);box off;xlabel('Time (s)');hold on
end
set(gcf,'Position',[250 150 300 350]);
box off, grid on;
h2 = legend([h(1) h(3) h(5) h(7) h(10)],'1','3','5','7','10','location','NorthEast');
set(h2,'box','off');ylabel('Acceleration')