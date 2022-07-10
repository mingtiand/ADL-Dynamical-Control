function [Ex Ey Ez Wx Wy Wz X Y Z q1m q2m q3m q4m q5m q6m q7m] = rg(O,Xt_x,Xt_y,Xt_z,alpha,lamda,q_initial,epsilon,fix)

    spine = 0;
    up = 40;
    fore = 35;%0.25 0.2
    palmar = 8;
    % L(1) = Link('d', 0, 'a', spine, 'alpha',0);
    % L(2) = Link('d', 0, 'a', 0, 'alpha',pi/2);
    % L(3) = Link('d', 0, 'a', 0, 'alpha',pi/2);% + pi/2
    % L(4) = Link('d', up, 'a', 0, 'alpha',0);
    % L(5) = Link('d', 0, 'a', 0, 'alpha',pi/2);% + pi/2
    % L(6) = Link('d', 0, 'a', -fore, 'alpha',0);
    % L(7) = Link('d', 0, 'a', 0, 'alpha',-pi/2);
    % L(8) = Link('d', 0, 'a', -palmar, 'alpha',0);
    % botPlot = SerialLink(L,'name','UpperLimb');
    % botPlot.teach
    % % syms q1 q2 q3 q4 q5 q6 q7
    % % q = [0 q1 q2+sym(pi/2) q3 q4+sym(pi/2) q5 q6 q7];
    % % fk = botPlot.fkine(q)
    % % H = [fk.n fk.o fk.a fk.t;0 0 0 1];
    syms q1 q2 q3 q4 q5 q6 q7
%     T10 = dh(0,0,0,q1);
%     T21 = dh(0,sym(pi/2),0,sym(q2+pi/2));
%     T32 = dh(0,sym(pi/2),0,q3);
%     T43 = dh(0,0,sym(up),sym(q4-pi/2));
%     T54 = dh(0,sym(-pi/2),0,q5);
%     T65 = dh(sym(fore),0,0,q6);
%     T76 = dh(0,sym(pi/2),0,q7); %T76 = dh(0,sym(-pi/2),0,q7);
%     TE7 = dh(sym(palmar),0,0,0);
    T10 = dh(0,0,0,q1);
    T21 = dh(0,sym(pi/2),0,sym(q2+pi/2));
    T32 = dh(0,sym(pi/2),sym(up),sym(q3-pi/2));
    T43 = dh(0,sym(-pi/2),0,sym(q4-pi/2));
    T54 = dh(0,sym(-pi/2),sym(fore),q5);
    T65 = dh(0,sym(pi/2),0,sym(q6+pi/2));
    T76 = dh(0,sym(pi/2),0,q7);
    TE7 = dh(sym(palmar),0,0,0);
    TE0 = T10*T21*T32*T43*T54*T65*T76*TE7;
    T30 = T10*T21*T32;
    fk = TE0*[0;0;0;1];
    fk_w = T10*T21*T32*T43*T54*[0;0;0;1];
    fk_e = T10*T21*T32*[0;0;0;1];
    H = TE0;
    H = simplify(H*dh(0,sym(-pi/2),0,sym(-pi/2)));
    R = inv(O)*H;
    T = simplify(fk,'steps',5);

    if fix ==1 || fix== 3
         alpha = 0;
    end

    F = sqrt((Xt_x - T(1))^2 + (Xt_y - T(2))^2 + (Xt_z - T(3))^2 + alpha*(acos(0.5*trace(R) - 1))^2);
    F = vpa(F);
    Fg = gradient(F,[q1 q2 q3 q4 q5 q6 q7]);
    %Fg = simplify(Fg);

    q = q_initial;

    X = [];
    Y = [];
    Z = [];
    Wx =[];Wy=[];Wz=[];
    Ex = [];Ey =[];Ez =[];
    bfk = subs(fk, [q1 q2 q3 q4 q5 q6 q7], double([q(1) q(2) q(3) q(4) q(5) q(6) q(7)]));
    bfk_w = subs(fk_w, [q1 q2 q3 q4 q5], double([q(1) q(2) q(3) q(4) q(5)]));
    bfk_e = subs(fk_e, [q1 q2 q3], double([q(1) q(2) q(3)]));
    X(1) = bfk(1);
    Y(1) = bfk(2);
    Z(1) = bfk(3);
    Wx(1) = bfk_w(1);
    Wy(1) = bfk_w(2);
    Wz(1) = bfk_w(3);
    Ex(1) = bfk_e(1);
    Ey(1) = bfk_e(2);
    Ez(1) = bfk_e(3);


    i = 2;

    q1m = [];q2m = [];q3m = [];q4m = [];q5m = [];q6m = [];q7m = [];
    q1m(i) = q(1);q2m(i) = q(2);q3m(i) = q(3);q4m(i) = q(4);q5m(i) = q(5);q6m(i) = q(6);q7m(i) = q(7);
    figure("Name","rg2 Trajectory")
    plot3(Xt_x,Xt_y,Xt_z,'o')
    hold on
    epsilon_before = double(subs(F,[q1 q2 q3 q4 q5 q6 q7],double([q(1) q(2) q(3) q(4) q(5) q(6) q(7)])));
    while double(abs(subs(F,[q1 q2 q3 q4 q5 q6 q7],q))) >epsilon
    
        dq1 = -double(subs(Fg(1),[q1 q2 q3 q4 q5 q6 q7],q));
        q_1 = q(1) + lamda*con2(1,q(1),dq1)*dq1;
    
        dq2 = -double(subs(Fg(2),[q1 q2 q3 q4 q5 q6 q7],q));
        c = con(q(2), -pi/2, dq2);
        q_2 = q(2) + lamda*c*con2(2,q(2),dq2)*dq2;
        %disp(lamda*c*dq2)
        dq3 = -double(subs(Fg(3),[q1 q2 q3 q4 q5 q6 q7],q));
        q_3 = q(3) + lamda*con2(3,q(3),dq3)*dq3;

        dq4 = -double(subs(Fg(4),[q1 q2 q3 q4 q5 q6 q7],q));
        q_4 = q(4) + lamda*con2(4,q(4),dq4)*dq4;
    
        dq5 = -double(subs(Fg(5),[q1 q2 q3 q4 q5 q6 q7],q));
        q_5 = q(5) + lamda*con2(5,q(5),dq5)*dq5;
        if fix==1 || fix == 2
            dq6 = -double(subs(Fg(6),[q1 q2 q3 q4 q5 q6 q7],q));
            q_6 = q(6) + lamda*con2(6,q(6),dq6)*dq6;

            dq7 = -double(subs(Fg(7),[q1 q2 q3 q4 q5 q6 q7],q));
            q_7 = q(7) + lamda*con2(7,q(7),dq7)*dq7;
        else
            q_6 = q(6);
            q_7 = q(7);

        end

        %disp('DEBUG1')
        q = [q_1 q_2 q_3 q_4 q_5 q_6 q_7];
        
        %C = diag([con(q(1), q_initial(1)), con(q(2), q_initial(2)), con(q(3), q_initial(3)), 1, 1]);
        epsilon_now = double(subs(F,[q1 q2 q3 q4 q5 q6 q7],q))
        if epsilon_now>epsilon_before || epsilon_before-epsilon_now<0.001
            break
        end
        epsilon_before = epsilon_now;
        
        %botPlot.plot(double([0 q_1 q_2+pi/2 q_3 q_4+pi/2 q_5 q_6 q_7]))
        bfk = subs(fk, [q1 q2 q3 q4 q5 q6 q7], q);
        bfk_w = subs(fk_w, [q1 q2 q3 q4 q5 q6 q7], q);
        bfk_e = subs(fk_e, [q1 q2 q3 q4 q5 q6 q7], q);

        X(i) = bfk(1);
        Y(i) = bfk(2);
        Z(i) = bfk(3);
        Wx(i) = bfk_w(1);
        Wy(i) = bfk_w(2);
        Wz(i) = bfk_w(3);
        Ex(i) = bfk_e(1);
        Ey(i) = bfk_e(2);
        Ez(i) = bfk_e(3);
        plot3([0 Ex(i) Wx(i)],[0 Ey(i) Wy(i)],[0 Ez(i) Wz(i)])
        hold on
        i = i+1;
        q1m(i) = q(1);q2m(i) = q(2);q3m(i) = q(3);q4m(i) = q(4);q5m(i) = q(5);q6m(i) = q(6);q7m(i) = q(7);
        
    end
    axis("square")
end


function T = dh(a,alpha,d,theta)
    Rx = [1 0 0 0;0 cos(alpha) -sin(alpha) 0;0 sin(alpha) cos(alpha) 0;0 0 0 1];
    Dx = [1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
    Rz = [cos(theta) -sin(theta) 0 0;sin(theta) cos(theta) 0 0;0 0 1 0;0 0 0 1];
    Dz = [1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];
    T = Rx*Dx*Rz*Dz;
end

function constraint = con(q_i, q_initial, dq)
    a_plus = 1; %1.75 1 1.5
    a_minus = 1; %0.05 1 0.5
    k = 25;
    constraint = a_plus + (a_minus - a_plus)/(1 + exp(-k*dq*(q_i - q_initial)));
    %disp(constraint)
end

function constraint2 = con2(i,q, dq)
    k = 25;
    if i==1 %flexion-extension
        q_max = pi/2;
        q_min = -pi/2;
    elseif i==2 %abduction-adduction
        q_max = pi/2;
        q_min = -pi/2;
    elseif i==3 %internal-external
        q_max = pi/2;
        q_min = -pi/2;
    elseif i==4 %elbow internal
        q_max = pi/2;
        q_min = -pi/2;
    elseif i==5 %elbow flexion
        q_max = pi/2;
        q_min = -pi/2;
    elseif i==6 %hand flexion
        q_max = pi/2;
        q_min = -pi/2;
    elseif i==7 %radial-Ulnar
        q_max = pi/4;
        q_min = -pi/4;
    end
        
    constraint2 = 1/(1+exp(k*dq))/(1+exp(q_max-q)) + ...
        1/(1+exp(-k*dq))/(1+exp(k*(q-q_min))) + ...
        1/(1+exp(-k*(q_max-q)))/(1+exp(-k*(q-q_min)));
    %constraint2 = 1;
end
