%% Figure 9, Convergence time and peak value comparison

clc;clear;close all;

N = 2:2:30;              % Number of vehicles in a platoon
error = [0,0.2,0.4,0.6];

EnergyT = zeros(length(N),length(error));
EnergyL = zeros(length(N),length(error));
C_N = zeros(length(N),length(error));
PeakV = zeros(length(N),length(error));

%% Time-domain simulation
for j = 1:length(error)
    for i = 1:length(N)
        [ EnergyT(i,j),EnergyL(i,j),C_N(i,j),PeakV(i,j) ] = AE( N(i), error(j));
    end
end

%% Figure
figure;
h1 = plot(N,C_N(:,1),'r*','MarkerSize',8,'linewidth',1.5);hold on; plot(N,C_N(:,1),'r-')
h2 = plot(N,C_N(:,2),'m.','MarkerSize',8,'linewidth',1.5);hold on; plot(N,C_N(:,2),'m-')
h3 = plot(N,C_N(:,3),'b>','MarkerSize',6,'linewidth',1.5);hold on; plot(N,C_N(:,3),'b-')
h4 = plot(N,C_N(:,4),'g<','MarkerSize',6,'linewidth',1.5);hold on; plot(N,C_N(:,4),'g-')
box off;xlabel('The number of vehicles:$N$','interpreter','latex');ylabel('Covergence Time','interpreter','latex')
h = legend([h1,h2,h3,h4],'$\epsilon=0$','$\epsilon=0.2$','$\epsilon=0.4$','$\epsilon=0.6$','location','NorthWest');
set(h,'box','off','interpreter','latex');
set(gcf,'Position',[250 150 400 340]);

figure;
h1 = plot(N,PeakV(:,1),'r*','MarkerSize',8,'linewidth',1.5);hold on; plot(N,PeakV(:,1),'r-')
h2 = plot(N,PeakV(:,2),'m.','MarkerSize',8,'linewidth',1.5);hold on; plot(N,PeakV(:,2),'m-')
h3 = plot(N,PeakV(:,3),'b>','MarkerSize',6,'linewidth',1.5);hold on; plot(N,PeakV(:,3),'b-')
h4 = plot(N,PeakV(:,4),'g<','MarkerSize',6,'linewidth',1.5);hold on; plot(N,PeakV(:,4),'g-')
box off;xlabel('The number of vehicles:N','interpreter','latex');ylabel('Peak Value','interpreter','latex')
h = legend([h1,h2,h3,h4],'$\epsilon=0$','$\epsilon=0.2$','$\epsilon=0.4$','$\epsilon=0.6$','location','NorthWest');
set(h,'box','off','interpreter','latex');
set(gcf,'Position',[250 150 400 340]);
