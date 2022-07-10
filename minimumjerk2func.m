function [EX1 EY1 EZ1 X1 Y1 Z1 t_sample t_joint] = minimumjerk2func(wpts,wpts_e)
    
    M = length(wpts(1,:));
    x = wpts(1,:);
    x = wpts(1,:) - wpts(1,1);
    y = wpts(2,:);
    y = wpts(2,:) - wpts(2,1);
    z = wpts(3,:);
    z = wpts(3,:) - wpts(3,1);
    %dis = sqrt((X(M)-X(1))^2 + (Y(M)-Y(1))^2 + (Z(M)-Z(1))^2); 
    dis = 0;
    for i = 1:M-1
        dis = dis + sqrt((x(i+1)-x(i))^2 + (y(i+1)-y(i))^2 + (z(i+1)-z(i))^2); 
    end
    %tpts = sqrt((X-X(1)).^2)./dis;
    syms t d
    d = 1;
    xyz = x(1) + (dis*(10*(t/d)^3 - 15*(t/d)^4 + 6*(t/d)^5));
    tpts = [];
    sum = 0;
    for i = 1:M
        
        if i>1
            sum = sum + sqrt((x(i)-x(i-1))^2 + (y(i)-y(i-1))^2 + (z(i)-z(i-1))^2);
        end
        T = solve(xyz == sum,t,'Real',true);
        T = simplify(T);
        Rnumeric = vpa(T);
        if(sum/dis < 0.5)
            Rnumeric = min(Rnumeric);
        else
            Rnumeric = max(Rnumeric);
        end
        tpts(i) = Rnumeric;
    
    end
%     v = diff(x,t);
%     V =[];
%     for i=1:length(tpts)
%         V(i) = subs(v,t,tpts(i))
%     end
%     figure(1)
%     plot(tpts,V)
%     x = x(1) + ((x(M)-x(1))*(10*(t/d)^3 - 15*(t/d)^4 + 6*(t/d)^5));
% 
%     subs(x,t,1)
    numsamples = 1000;
    [q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(x,tpts,numsamples);
    %figure("Name","x_velocity")
    X = [q + wpts(1,1);qd;qdd;qddd];
    %plot(tsamples,qd)
    [q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(y,tpts,numsamples);
    Y = [q + wpts(2,1);qd;qdd;qddd];
    %figure("Name","y_velocity")
    %plot(tsamples,qd)
    [q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(z,tpts,numsamples);
    Z = [q + wpts(3,1);qd;qdd;qddd];


    [q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(wpts_e(1,:),tpts,numsamples);
    EX = [q;qd;qdd;qddd];
    [q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(wpts_e(2,:),tpts,numsamples);
    EY = [q;qd;qdd;qddd];
    [q,qd,qdd,qddd,pp,timepoints,tsamples] = minjerkpolytraj(wpts_e(3,:),tpts,numsamples);
    EZ = [q;qd;qdd;qddd];
    %figure("Name","z_velocity")
    %plot(tsamples,qd)
    t_sample = tsamples;
    t_joint = tpts;
            X1(1,1) = X(1,1);Y1(1,1) = Y(1,1);Z1(1,1) = Z(1,1);t_joint(1,1) = tsamples(1,1);
        EX1(1,1) = EX(1,1);EY1(1,1) = EY(1,1);EZ1(1,1) = EZ(1,1);
    for i = 0:1:20
        N = 50*i;
    if(i~=0)
        X1(1,i+1) = X(1,N);Y1(1,i+1) = Y(1,N);Z1(1,i+1) = Z(1,N);t_joint(1,i+1) = tsamples(1,N);
        EX1(1,i+1) = EX(1,N);EY1(1,i+1) = EY(1,N);EZ1(1,i+1) = EZ(1,N);
        %X2(1,i+1) = X22(1,N);Y2(1,i+1) = Y22(1,N);Z2(1,i+1) = Z22(1,N);t2(1,i+1) = t22(1,N);
    end
    end
end


