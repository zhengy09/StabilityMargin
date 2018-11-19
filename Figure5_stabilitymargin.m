%% Figure 5: Stability margin
% Bidrection Topology
% M = L+P

clc;clear;close all

%% Parameter initialization
N = 50;                            % Number of vehicles
StabilityMargin = zeros(N,N-1);    % Stability Margin

%% Comparsion 

for Ileader = 1:N  % Ileader == c(parameter in paper)

    for k = 1:N-1 % the number of off-diagonal elements

        M = zeros(N); %% matrix L 
        for i = 1:N
            for j = i+1:N
                if j <= i + k && j ~= i 
                    M(i,j) = -1;
                    M(j,i) = -1;
                end
            end
        end

        for i = 1:N
            M(i,i) = -sum(M(i,:));
        end

        M1 = M;      %% matrix L+P 
        for i = 1:N
            if mod(i,Ileader)+1==1
                M1(i,i) = M1(i,i)+1;
            end  
        end
        StabilityMargin(Ileader,k) = min(eig(M1));
    end
end

close all; 
figure
mesh(StabilityMargin);
set(gca,'ydir','reverse');
ylabel('Tree Depth Parameter:c');xlabel('Local Communication Range:k')
zlabel('$\sigma_{\min}(L+P)$','interpreter','latex')
set(gcf,'Position',[250 150 550 400]);
ylim([0 50])

%%
tao = 0.5;
k1 = 1;
k2 = 2;
k3 = 3;
SM = zeros(N,N-1);

for Ileader = 1:N
    for k = 1:N-1
        C = [1 (StabilityMargin(Ileader,k)*k3+1)/tao (StabilityMargin(Ileader,k)*k2)/tao (StabilityMargin(Ileader,k)*k1)/tao];
        SM(Ileader,k) = max(real(roots(C)));
    end
end
figure
mesh(-SM);
set(gca,'ydir','reverse');
ylabel('Tree Depth Parameter:c');xlabel('Local Communication Range:h')
zlabel('Stability Margin')
set(gcf,'Position',[250 150 500 400]);
ylim([0 50])

figure;
surf(-SM);
set(gca,'ydir','reverse');
ylabel('Tree Depth Parameter:c');xlabel('Local Communication Range:h')
zlabel('Stability Margin')
set(gcf,'Position',[250 150 550 400]);
ylim([0 50])

