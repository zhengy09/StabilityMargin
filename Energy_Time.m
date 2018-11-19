function [T_N,L_N,C_N ] = Energy_Time( Ileader )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    cnt = 0;
    N = [2:4:50];  
    T_N = zeros(length(N),1);
    L_N = zeros(length(N),1);
    C_N = zeros(length(N),1);

    for N = [2:4:50]           % Number of vheicles

        cnt = cnt + 1;
        count = 0;
        Time = 1000;     % Total time
        T_delta = 0.01;  % Step size

        Tao = 0.5;       % tiemlag

        %% state of the platoon

        X = zeros(Time/T_delta,N);
        V = zeros(Time/T_delta,N);
        a = zeros(Time/T_delta,N);
        U = zeros(Time/T_delta,N);

        d = 20;   % desired spacing
        %% leader's state
        a0 = zeros(Time/T_delta,1);
        v0 = zeros(Time/T_delta,1);
        x0 = zeros(Time/T_delta,1);

          %% desired acceleration
         a0(5/T_delta:10/T_delta) = 2;
         v0(1) = 20;

        for i = 1:N
            X(1,i) = x0(1)-i*d;    
            V(1,i) = 20;            
        end


        %% updating leader
        for i = 2:Time/T_delta
            v0(i) = v0(i-1)+a0(i)*T_delta;
            x0(i) = x0(i-1)+v0(i)*T_delta;    
        end



        %% 队列状态更新

        % feedback gain =
         k1 = 1; k2 = 2; k3 = 1;    

        for i = 2:Time/T_delta




            for j = 1:N

                Ex = 0;Ev = 0; Ea = 0;        
                for jj = 1:N
                        Ex = Ex + X(i-1,j)-X(i-1,jj) - (jj-j)*d;
                        Ev = Ev + V(i-1,j)-V(i-1,jj);
                        Ea = Ea + a(i-1,j)-a(i-1,jj);
                end


                 if  mod(j,Ileader) ==1
                        Ex = Ex + X(i-1,j)-x0(i-1) + j*d;
                        Ev = Ev + V(i-1,j)-v0(i-1);
                        Ea = Ea + a(i-1,j)-a0(i-1);
                 end     

                U(i,j) = - k1*Ex - k2*Ev - k3*Ea;
                a(i,j) = (Tao-T_delta)/Tao*a(i-1,j) + T_delta/Tao*U(i,j);
                V(i,j) = V(i-1,j)+a(i,j)*T_delta;
                X(i,j) = X(i-1,j)+V(i,j)*T_delta; 
            end

            if count < 1000 && i > 5/T_delta
                delta_x = zeros(N,1);
                delta_x(1) = abs(X(i,1)-x0(i) + d);
                for jj = 2:N
                    delta_x(jj) = abs(X(i,jj)-X(i,jj-1) + d);
                end
                if max(delta_x) < 0.1
                    count = count + 1;
                else
                    count = 0;  
                end
                if count > 999
                    C_N(cnt) =(i-count)*T_delta;
                end
            end
            % 终止
            if i*T_delta > C_N(cnt) + 50 && C_N(cnt) > 1          
                Time_c = i-10;
                break;
            end

        end

        EnergyT = 0;
        EnergyL = 0;

        for i = 2:Time_c
                delta_a = a(i,1) - a0(i);
                delta_X = X(i,1) - x0(i) +d; 
                delta_V = V(i,1) - v0(i);
                EnergyT = EnergyT + 1/2*(k1*delta_X^2+k2*delta_V^2+k3*delta_a^2)*T_delta;  

            for j = 2:N
                delta_a = a(i,j) - a(i,j-1);
                delta_X = X(i,j) - X(i,j-1) +d; 
                delta_V = V(i,j) - V(i,j-1);

                EnergyT = EnergyT + 1/2*(k1*delta_X^2+k2*delta_V^2+k3*delta_a^2)*T_delta;  
                EnergyL = EnergyL + 1/2*(k1*delta_X^2+k2*delta_V^2+k3*delta_a^2)*T_delta; 

            end  
        end

        T_N(cnt) = EnergyT/N;
        L_N(cnt) = EnergyL/N;


    end


end

