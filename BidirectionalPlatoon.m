%% Bidirectional platoon, simulations


clc;clear;close all;


N = 10;           % Number of vehicles
Time = 100;       % Total time
T_delta = 0.01;   % Time step

Tao = 0.5;        % Time lag

%% State of individual vehicles

X = zeros(Time/T_delta,N);
V = zeros(Time/T_delta,N);
a = zeros(Time/T_delta,N);
U = zeros(Time/T_delta,N);

d = 10;   % desired spacing
%% leader's state
a0 = zeros(Time/T_delta,1);
v0 = zeros(Time/T_delta,1);
x0 = zeros(Time/T_delta,1);

  %% the leader's constant acceleration 
 a0(5/T_delta:10/T_delta) = 2;
 v0(1) = 20;

for i = 1:N
    X(1,i) = x0(1)-i*d;
    V(1,i) = 20;
end


%% update leader's state
for i = 2:Time/T_delta
    v0(i) = v0(i-1)+a0(i)*T_delta;
    x0(i) = x0(i-1)+v0(i)*T_delta;    
end



%% update platoon state

% feedback gains
% k1 = 1; k2 = 2; k3 = 1;    % stable
   k1 = 1; k2 = 0.2; k3 = 1;  % unstable 

for i = 2:Time/T_delta
    
    %% first vehicle
   % symmetric 
    U(i,1) = - k1*(X(i-1,1)-x0(i-1)+d) - k2*(V(i-1,1)-v0(i-1))-k3*(a(i-1,1)-a0(i-1))-...
             k1*(X(i-1,1)-X(i-1,2)-d) - k2*(V(i-1,1)-V(i-1,2))-k3*(a(i-1,1)-a(i-1,2));%+ 0.1*sin(1.44*(T_delta*i));   
           
    a(i,1) = (Tao-T_delta)/Tao*a(i-1,1) + T_delta/Tao*U(i,1);
    V(i,1) = V(i-1,1)+a(i,1)*T_delta;
    X(i,1) = X(i-1,1)+V(i,1)*T_delta; 
  
    %% j-th vheicle
    for j = 2:N-1
        % symmetric case
        U(i,j) = - k1*(X(i-1,j)-X(i-1,j-1)+d) - k2*(V(i-1,j)-V(i-1,j-1))-k3*(a(i-1,j)-a(i-1,j-1)) - ...
                   k1*(X(i-1,j)-X(i-1,j+1)-d) - k2*(V(i-1,j)-V(i-1,j+1))-k3*(a(i-1,j)-a(i-1,j+1)) - ...
                    k1*(X(i-1,j)-x0(i-1)+j*d) - k2*(V(i-1,j)-v0(i-1))-k3*(a(i-1,j)-a0(i-1)); 
        a(i,j) = (Tao-T_delta)/Tao*a(i-1,j) + T_delta/Tao*U(i,j);
        V(i,j) = V(i-1,j)+a(i,j)*T_delta;
        X(i,j) = X(i-1,j)+V(i,j)*T_delta; 
    end
    
    U(i,N) = - k1*(X(i-1,N)-X(i-1,N-1)+d) - k2*(V(i-1,N)-V(i-1,N-1))-k3*(a(i-1,N)-a(i-1,N-1));            
        a(i,N) = (Tao-T_delta)/Tao*a(i-1,N) + T_delta/Tao*U(i,N);
        V(i,N) = V(i-1,N)+a(i,N)*T_delta;
        X(i,N) = X(i-1,N)+V(i,N)*T_delta; 
end

%% figure

t = T_delta:T_delta:Time;
ColorSet = {'r','b','k','g','m','r-.','b-.','k-.','g-.','m-.'};

h = zeros(10,1);
figure;
h(1) = plot(t,x0 - X(:,1) -d,'r','linewidth',2);hold on;

% spacing error
for i = 1:N-1
   h(i+1) = plot(t,X(:,i)-X(:,i+1)-d,ColorSet{i+1},'linewidth',2);box off;xlabel('Time (s)');hold on
end
set(gcf,'Position',[250 150 300 350]);
%    ylim([-20 60])
%   ylim([-1 3]);
% xlim([0 300])
box off, grid on;
% title('Bidirectional - leader structure')
  %h = legend('1','2','3','4','5','6','7','8','9','10','location','NorthWest');ylabel('Spacing Error(m)')
  h1 = legend([h(1) h(3) h(5) h(7) h(10)],'1','3','5','7','10','location','NorthWest');ylabel('Spacing Error (m)')
% h1 = legend([h(1) h(3) h(5) h(7) h(10)],'1','3','5','7','10','location','NorthEast');ylabel('Spacing Error (m)')
set(h1,'box','off')