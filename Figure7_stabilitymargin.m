%% Figure 7, sclaing of stability margin 
% Bidirectional + different position of leaders

clc;clear;close all

N = 2:4:200;     %% number of vehicles --> different size of matrices

Min2 = zeros(length(N),1);
Min3 = zeros(length(N),1);
Min4 = zeros(length(N),1);
MinN2 = zeros(length(N),1);
MinN3 = zeros(length(N),1);
MinN4 = zeros(length(N),1);

for j = 1:length(N)
    
    %% basic L+P matrix
    M = zeros(N(j),N(j));  
    M(N(j),N(j)-1) = -1; M(1,2) = -1;
    for i = 2:N(j)-1
       M(i,i+1) = -1;
       M(i,i-1) = -1;
    end
    for i = 1:N(j)
        M(i,i) = -sum(M(i,:));
    end

    M2 = M;       %% L+P matrix
    M3 = M;       %% L+P matrix
    M4 = M;       %% L+P matrix
    M2N = M;      %% L+P matrix
    M3N = M;      %% L+P matrix
    M4N = M;      %% L+P matrix
    
    %% Different connections with the leader
        for i = 1:N(j)
            if mod(i,2)==1   % case 1: one leader every two vehicles
                M2(i,i) = M(i,i)+1;
            end
            if mod(i,3)==1   % case 2: one leader every three vehicles
                M3(i,i) = M(i,i)+1;
            end 
            if mod(i,4)==1   % case 3: one leader every four vehicles
                M4(i,i) = M(i,i)+1;
            end 
            if i <= ceil(N(j)/2)  % case 4: 
                M2N(i,i) = M(i,i)+1;
            end
            if i <= ceil(N(j)/3)  % case 5: 
                M3N(i,i) = M(i,i)+1;
            end
            if i <= ceil(N(j)/4)  % case 6: 
                M4N(i,i) = M(i,i)+1;
            end
        end
        
        %% computing minimum eigenvalue
        Min2(j) = min(eig(M2));
        Min3(j) = min(eig(M3));
        Min4(j) = min(eig(M4));
        MinN2(j) = min(eig(M2N));
        MinN3(j) = min(eig(M3N));
        MinN4(j) = min(eig(M4N));
end

close all;
figure;
loglog(N,Min2,'r*');hold on;
loglog(N,Min4,'b>');
loglog(N,MinN2,'g.');hold on;
loglog(N,MinN4,'md');hold on;

% title('L+G for bidirectional structure')
box off;xlabel('The number of vehicles:N');ylabel('考_m_i_n(L+P) ')
h = legend('次(N)=N/2,Tree Depth: 2','次(N)=N/4,Tree Depth: 4','次(N)=N/2,Tree Depth: N/2','次(N)=N/4,Tree Depth: 3N/4','location','SouthWest');
set(h,'box','off');
set(gcf,'Position',[250 150 360 340]);
ylim([10^(-6) 1])
% set(gca,'YTick',[10^(-6),10^(-5),10^(-4),10^(-3),10^(-2),10^(-1),1,10])

%%

SM1 = zeros(length(N),1);
SM2 = zeros(length(N),1);
SM3 = zeros(length(N),1);
SM4 = zeros(length(N),1);

tao = 0.5;
k1 = 1;
k2 = 2;
k3 = 3;

for i = 1:length(N)
    C = [1 (Min2(i)*k3+1)/tao (Min2(i)*k2)/tao (Min2(i)*k1)/tao];
    SM1(i) = max(real(roots(C)));
    
    C = [1 (Min4(i)*k3+1)/tao (Min4(i)*k2)/tao (Min4(i)*k1)/tao];
    SM2(i) = max(real(roots(C)));
    
    C = [1 (MinN2(i)*k3+1)/tao (MinN2(i)*k2)/tao (MinN2(i)*k1)/tao];
    SM3(i) = max(real(roots(C)));
    
    C = [1 (MinN4(i)*k3+1)/tao (MinN4(i)*k2)/tao (MinN4(i)*k1)/tao];
    SM4(i) = max(real(roots(C)));
end

%% Plot

Index = zeros(30,1);
Range = (log10(max(N))-log10(2))/30;
for i = 1:31
    temp = log10(8)+(i-1)*Range;
    Index(i) = find(abs(N-10^(temp)) == min(abs(N-10^(temp))));
end

figure;
loglog(N(Index),-SM1(Index),'r*','MarkerSize',8,'linewidth',1.5);hold on;
loglog(N(Index),-SM2(Index),'m.','MarkerSize',16,'linewidth',1.5); hold on;
loglog(N(Index),-SM3(Index),'b<','MarkerSize',6,'linewidth',1.5);
loglog(N(Index),-SM4(Index),'gs','MarkerSize',6,'linewidth',1.5);hold on;

box off;xlabel('The number of vehicles:N');ylabel('Stability Margin ')
h = legend('次(N)=N/2,Tree depth: c=2','次(N)=N/4,Tree depth: c=4','次(N)=N/2,Tree depth: c=N/2','次(N)=N/4,Tree depth: c=3N/4','location','SouthWest');
set(h,'box','off');
set(gcf,'Position',[250 150 360 340]);
xlim([5 500])
ylim([10^(-6) 1])
set(gca,'XTick',[10 100 500])

