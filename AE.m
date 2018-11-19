function [EnergyT,EnergyL,C_N,PeakV] = AE( N, error)
% Time domain simulaiton

PeakV = 0;
C_N   = 0;
count = 0;

   %%
    Time = 2000;     % Total simulation
    T_delta = 0.01;  % step size

    Tao = 0.5;       % Tau lag

    %% State of vehicles in a platoon

    X = zeros(Time/T_delta,N);
    V = zeros(Time/T_delta,N);
    a = zeros(Time/T_delta,N);
    U = zeros(Time/T_delta,N);

    d = 10;   % constant desired spacing
    %% leader's state
    a0 = zeros(Time/T_delta,1);
    v0 = zeros(Time/T_delta,1);
    x0 = zeros(Time/T_delta,1);

    %% constant acceleration
     a0(5/T_delta:10/T_delta) = 2;
     v0(1) = 20;

    for i = 1:N
        X(1,i) = x0(1)-i*d;% - (rand(1)-1)*5;
        V(1,i) = 20;% - (rand(1)-1)*2;
    end


    %% update leader's state
    for i = 2:Time/T_delta
        v0(i) = v0(i-1)+a0(i)*T_delta;
        x0(i) = x0(i-1)+v0(i)*T_delta;    
    end

    %% Update the vehicles in the platoon

    % feedback gain
    k1 = 1; k2 = 2; k3 = 1;        % Stable
    %  k1 = 1; k2 = 0.2; k3 = 1;  % Unstable


    for i = 2:Time/T_delta

        %% First vehicle
       % symmetric
        U(i,1) = - (1+error)*k1*(X(i-1,1)-x0(i-1)+d) - (1+error)*k2*(V(i-1,1)-v0(i-1))-(1+error)*k3*(a(i-1,1)-a0(i-1))-...
                 (1-error)*k1*(X(i-1,1)-X(i-1,2)-d) - (1-error)*k2*(V(i-1,1)-V(i-1,2))-(1-error)*k3*(a(i-1,1)-a(i-1,2));%+ 0.1*sin(1.44*(T_delta*i));   

        a(i,1) = (Tao-T_delta)/Tao*a(i-1,1) + T_delta/Tao*U(i,1);
        V(i,1) = V(i-1,1)+a(i,1)*T_delta;
        X(i,1) = X(i-1,1)+V(i,1)*T_delta; 

        %% j-th vehicle
        for j = 2:N-1
            % symmetric weight
            U(i,j) = - (1+error)*k1*(X(i-1,j)-X(i-1,j-1)+d) - (1+error)*k2*(V(i-1,j)-V(i-1,j-1))-(1+error)*k3*(a(i-1,j)-a(i-1,j-1)) - ...
                       (1-error)*k1*(X(i-1,j)-X(i-1,j+1)-d) - (1-error)*k2*(V(i-1,j)-V(i-1,j+1))-(1-error)*k3*(a(i-1,j)-a(i-1,j+1));

            a(i,j) = (Tao-T_delta)/Tao*a(i-1,j) + T_delta/Tao*U(i,j);
            V(i,j) = V(i-1,j)+a(i,j)*T_delta;
            X(i,j) = X(i-1,j)+V(i,j)*T_delta; 
        end

        U(i,N) = - (1+error)*k1*(X(i-1,N)-X(i-1,N-1)+d) - (1+error)*k2*(V(i-1,N)-V(i-1,N-1))-(1+error)*k3*(a(i-1,N)-a(i-1,N-1));            
            a(i,N) = (Tao-T_delta)/Tao*a(i-1,N) + T_delta/Tao*U(i,N);
            V(i,N) = V(i-1,N)+a(i,N)*T_delta;
            X(i,N) = X(i-1,N)+V(i,N)*T_delta; 

            if count < 1000 && i > 5/T_delta
                    delta_x = zeros(N,1);
                    delta_x(1) = abs(X(i,1)-x0(i) + d);
                    for jj = 2:N
                        delta_x(jj) = abs(X(i,jj)-X(i,jj-1) + d);
                    end
                    if max(delta_x) > PeakV
                        PeakV =  max(delta_x);
                    end
                    if max(delta_x) < 0.1
                        count = count + 1;
                    else
                        count = 0;  
                    end
                    if count > 999
                        C_N =(i-count)*T_delta - 5;
                    end
                end
                % end
                if i*T_delta > C_N + 50 && C_N > 1          
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


end

