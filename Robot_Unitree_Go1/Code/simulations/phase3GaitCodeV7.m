clc; clear; close all;

% Define Gates Cycles with phase and beta (duty factor)
%convention: LF RF RR LR
% phi(1,:) = [0, 0.25, 0.5, 0.75]; % Crawl
phi(1,:) = [0, 0.25, 0.5, 0.75]; % Walk 
phi(2,:) = [0, 0.5, 0, 0.5]; % Trot
% phi(4,:) = [0, 0.5, 0.5, 0]; % Pace
% phi(5,:) = [0, 0, 0.5, 0.5]; % Bound
phi(3,:) = [0.69, 0.7, 0.18, 0.20]; % Gallop
phi(4,:) = [0.1,0.25,0.6,0.35];%Canter

% beta = [0.9, 0.75, 0.5, 0.5, 0.5, 0.35];
beta = [0.75, 0.5, 0.40];
betaCanter = [0.5,0.35,0.4,0.75]; %Canter
% beta = stance factor (percent time in stance)
% titles = ["Crawl", "Walk", "Trot", "Pace", "Bound", "Gallop"];
titles = ["Walk", "Trot", "Gallop"];

swingFrac = 1 - beta;
swingFracCanter = 1 - betaCanter;


%% Frw Kinemetics to get Y Terr Val
a2 = 0.213;
a3 = 0.212272;
% radius = 0.022921 % r of spherical foot
d1 = 0.0601;
d2 = 0.081;


%% test the standing position position
% L1 = Link('revolute', 'd', 0, 'a', a2, 'alpha', 0);
% L2 = Link('revolute', 'd', 0, 'a', a3, 'alpha', 0);
% bot = SerialLink([L1 L2], 'name', 'stand pos');
% figure();
% bot.plot([0.67-pi/2, -1.3]) % -pi/2 to rotate the unit circle
% bot.fkine([0.67-pi/2, -1.3]).t
%% Gait Path 

%Chosen Values
Vcm = [0.5,1,0.7]; %m/s
gaitTime = [1.0,0.70,1.2]; %seconds

%Since cnater and gallop are so similar, V,t, and l are the same
Vcanter = Vcm(3);
gaitTimeCanter = gaitTime(3);

% a faster gait time means short lStance
lStance = Vcm .* (gaitTime .* (1 - swingFrac)); % d = v*t
lStanceCanter = lStance(3);

hMax = 0.20; %max height end effector swings to (meters)

th2Max = 0.67 - pi/2; % taken from servo.cpp, 
% The -pi/2 rotates (0,0) of the unit circle to point down instead of right
th3Max = -1.3;

yEF = 0; % setting to to 0 cause I got the math to work it like this

% Instead, lets say that the standing height is the y-location of joint2
% we can find that using the stand angles given by servo.cpp (stand file)
end_eff_y_pos = a2.*sin(th2Max)+a3.*sin(th2Max+th3Max);
% We use cos above because the negative angle caused a sin flip in cos/sin
% I don't really know, I just know sin didn't work and this did.

% The math I did later in this script wants joint position, not end-eff
% so this converts a negative height to the positive joint position 
% assuming ground is 0
joint_height_from_ground = 0 - end_eff_y_pos;


n = 100.*gaitTime; % number of time steps in 1 gait cycle
n = round(n);
nC = n(3);
% round to avoid double precision errors 
% The int above in the calculation heavily impacts many calculations later

Walk.xLF = zeros(1,n(1));
Walk.xRF = zeros(1,n(1));
Walk.xRR = zeros(1,n(1));
Walk.xLR = zeros(1,n(1));
Walk.yLF = zeros(1,n(1));
Walk.yRF = zeros(1,n(1));
Walk.yRR = zeros(1,n(1));
Walk.yLR = zeros(1,n(1));

Trot.xLF = zeros(1,n(2));
Trot.xRF = zeros(1,n(2));
Trot.xRR = zeros(1,n(2));
Trot.xLR = zeros(1,n(2));
Trot.yLF = zeros(1,n(2));
Trot.yRF = zeros(1,n(2));
Trot.yRR = zeros(1,n(2));
Trot.yLR = zeros(1,n(2));

Gallop.xLF = zeros(1,n(3));
Gallop.xRF = zeros(1,n(3));
Gallop.xRR = zeros(1,n(3));
Gallop.xLR = zeros(1,n(3));
Gallop.yLF = zeros(1,n(3));
Gallop.yRF = zeros(1,n(3));
Gallop.yRR = zeros(1,n(3));
Gallop.yLR = zeros(1,n(3));

Canter.xLF = zeros(1,nC);
Canter.xRF = zeros(1,nC);
Canter.xRR = zeros(1,nC);
Canter.xLR = zeros(1,nC);
Canter.yLF = zeros(1,nC);
Canter.yRF = zeros(1,nC);
Canter.yRR = zeros(1,nC);
Canter.yLR = zeros(1,nC);

for i = 1:length(lStance)

    x(i,:) = [0,-0.08*lStance(i),-0.05*lStance(i),lStance(i)/2,1.05*lStance(i),1.08*lStance(i),lStance(i)];
    y(i,:) = [0,0.2*hMax,0.6*hMax,hMax,0.6*hMax,0.2*hMax,0];
    xy = [x(i,:);y(i,:)];
    spline(i) = cscvn(xy);
    breaks = fnbrk(spline(i),'breaks');
    ts(i) = breaks(1);
    tf(i) = breaks(end);
    t = linspace(ts(i),tf(i),n(i)*(1-beta(i)));
    points = fnval(spline(i),t);
    xSwing = points(1,:);
    ySwing = points(2,:);

%     figure(i)
%     plot(xSwing,ySwing,'k-','linewidth',3)
%     hold on
%     fnplt(cscvn(xy),'r',2)   
%     hold on
%     plot([0,lStance(i)],[0,0])
%     axis equal
%     title(sprintf('%s: - Swing Percentage = %.2f, Velocity = %.2f, Gait Cycle Time = %.2f, Length of Stance Phase', titles(i), swingFrac(i), Vcm(i), gaitTime(i)));


    tSwing(i) = (1-beta(i))*gaitTime(i);
    tStance(i) = (beta(i)).*gaitTime(i);
    xStance = linspace(lStance(i),0,beta(i)*n(i));
    yStance = zeros(1,beta(i)*n(i));

    xTotal = [xSwing,xStance];
    yTotal = [ySwing,yStance];
    
    swingTriggerLF(i) = round(n(i)*phi(i,1));
    swingTriggerRF(i) = round(n(i)*phi(i,2));
    swingTriggerRR(i) = round(n(i)*phi(i,3));
    swingTriggerLR(i) = round(n(i)*phi(i,4));

    if i == 1   % Walk Gait

        Walk.time = linspace(0,gaitTime(i),length(xSwing)+n(i));
        Walk.xLF = circshift(xTotal,swingTriggerLF(i));
        Walk.yLF = circshift(yTotal,swingTriggerLF(i));
        Walk.yLF = yEF - Walk.yLF;

        Walk.xRF = circshift(xTotal,swingTriggerRF(i));
        Walk.yRF = circshift(yTotal,swingTriggerRF(i));
        Walk.yRF = yEF - Walk.yRF;

        Walk.xRR = circshift(xTotal,swingTriggerRR(i));
        Walk.yRR = circshift(yTotal,swingTriggerRR(i));
        Walk.yRR = yEF - Walk.yRR;

        Walk.xLR = circshift(xTotal,swingTriggerLR(i));
        Walk.yLR = circshift(yTotal,swingTriggerLR(i));
        Walk.yLR = yEF - Walk.yLR;

        
    elseif i == 2  %Trot Gait

        Trot.time = linspace(0,gaitTime(i),length(xSwing)+n(i));
        Trot.xLF = circshift(xTotal,swingTriggerLF(i));
        Trot.yLF = circshift(yTotal,swingTriggerLF(i));
        Trot.yLF = yEF - Trot.yLF;

        Trot.xRF = circshift(xTotal,swingTriggerRF(i));
        Trot.yRF = circshift(yTotal,swingTriggerRF(i));
        Trot.yRF = yEF - Trot.yRF;

        Trot.xRR = circshift(xTotal,swingTriggerRR(i));
        Trot.yRR = circshift(yTotal,swingTriggerRR(i));
        Trot.yRR = yEF - Trot.yRR;

        Trot.xLR = circshift(xTotal,swingTriggerLR(i));
        Trot.yLR = circshift(yTotal,swingTriggerLR(i));
        Trot.yLR = yEF - Trot.yLR;
        
    elseif i == 3  %Gallop Gait
        Gallop.time = linspace(0,gaitTime(i),length(xSwing)+n(i));
        Gallop.xLF = circshift(xTotal,swingTriggerLF(i));
        Gallop.yLF = circshift(yTotal,swingTriggerLF(i));
        Gallop.yLF = yEF - Gallop.yLF;

        Gallop.xRF = circshift(xTotal,swingTriggerRF(i));
        Gallop.yRF = circshift(yTotal,swingTriggerRF(i));
        Gallop.yRF = yEF - Gallop.yRF;

        Gallop.xRR = circshift(xTotal,swingTriggerRR(i));
        Gallop.yRR = circshift(yTotal,swingTriggerRR(i));
        Gallop.yRR = yEF - Gallop.yRR;

        Gallop.xLR = circshift(xTotal,swingTriggerLR(i));
        Gallop.yLR = circshift(yTotal,swingTriggerLR(i));
        Gallop.yLR = yEF - Gallop.yLR;
    end

end

%% Canter Gait

%Constance between each duty cycle for canter
    xC = [0,-0.08*lStanceCanter,-0.05*lStanceCanter,lStanceCanter/2,1.05*lStanceCanter,1.08*lStanceCanter,lStanceCanter];
    yC = [0,0.2*hMax,0.6*hMax,hMax,0.6*hMax,0.2*hMax,0];
    xyC = [xC;yC];
    splineC = cscvn(xyC);
    breaksC = fnbrk(splineC,'breaks');
    tsC = breaksC(1);
    tfC = breaksC(end);

for i = 1:length(betaCanter)
    tC = linspace(tsC,tfC,nC*(1-betaCanter(i)));
    pointsC = fnval(splineC,tC);
    xSwingC = pointsC(1,:);
    ySwingC = pointsC(2,:);

%     figure(i)
%     plot(xSwing,ySwing,'k-','linewidth',3)
%     hold on
%     fnplt(cscvn(xy),'r',2)   
%     hold on
%     plot([0,lStance(i)],[0,0])
%     axis equal
%     title(sprintf('%s: - Swing Percentage = %.2f, Velocity = %.2f, Gait Cycle Time = %.2f, Length of Stance Phase', titles(i), swingFrac(i), Vcm(i), gaitTime(i)));


    tSwingC(i) = (1-betaCanter(i))*gaitTimeCanter;
    tStanceC(i) = (betaCanter(i)).*gaitTimeCanter;
    xStanceC = linspace(lStanceCanter,0,betaCanter(i)*nC);
    yStanceC = zeros(1,betaCanter(i)*nC);

    xTotalC = [xSwingC,xStanceC];
    yTotalC = [ySwingC,yStanceC];
    
    swingTriggerLFC = round(nC*phi(4,1));
    swingTriggerRFC = round(nC*phi(4,2));
    swingTriggerRRC = round(nC*phi(4,3));
    swingTriggerLRC = round(nC*phi(4,4));

    

        Canter.time = linspace(0,gaitTimeCanter,length(xSwingC)+nC);
        Canter.xLF = circshift(xTotalC,swingTriggerLFC);
        Canter.yLF = circshift(yTotalC,swingTriggerLFC);
        Canter.yLF = yEF - Canter.yLF;

        Canter.xRF = circshift(xTotalC,swingTriggerRFC);
        Canter.yRF = circshift(yTotalC,swingTriggerRFC);
        Canter.yRF = yEF - Canter.yRF;

        Canter.xRR = circshift(xTotalC,swingTriggerRRC);
        Canter.yRR = circshift(yTotalC,swingTriggerRRC);
        Canter.yRR = yEF - Canter.yRR;

        Canter.xLR = circshift(xTotalC,swingTriggerLRC);
        Canter.yLR = circshift(yTotalC,swingTriggerLRC);
        Canter.yLR = yEF - Canter.yLR;
        
   

end

%% Adjust back to joint 2

% Now this generates a curve starting on (0,0) and reaching to -hMax with 
% width of lStance
% To correctly model a leg, we need (0,0) centered on the joint (the middle
% of lStance) and also the flat part to be at -hMax instead of 0.
Walk.xRF = Walk.xRF - lStance(1)/2;
Walk.yRF = -(Walk.yRF)-hMax;
Walk.xLF = Walk.xLF - lStance(1)/2;
Walk.yLF = -(Walk.yLF)-hMax;
Walk.xRR = Walk.xRR - lStance(1)/2;
Walk.yRR = -(Walk.yRR)-hMax;
Walk.xLR = Walk.xLR - lStance(1)/2;
Walk.yLR = -(Walk.yLR)-hMax;

Trot.xRF = Trot.xRF - lStance(2)/2;
Trot.yRF = -(Trot.yRF)-hMax;
Trot.xLF = Trot.xLF - lStance(2)/2;
Trot.yLF = -(Trot.yLF)-hMax;
Trot.xRR = Trot.xRR - lStance(2)/2;
Trot.yRR = -(Trot.yRR)-hMax;
Trot.xLR = Trot.xLR - lStance(2)/2;
Trot.yLR = -(Trot.yLR)-hMax;

Gallop.xRF = Gallop.xRF - lStance(3)/2;
Gallop.yRF = -(Gallop.yRF)-hMax;
Gallop.xLF = Gallop.xLF - lStance(3)/2;
Gallop.yLF = -(Gallop.yLF)-hMax;
Gallop.xRR = Gallop.xRR - lStance(3)/2;
Gallop.yRR = -(Gallop.yRR)-hMax;
Gallop.xLR = Gallop.xLR - lStance(3)/2;
Gallop.yLR = -(Gallop.yLR)-hMax;

Canter.xRF = Canter.xRF - lStanceCanter/2;
Canter.yRF = -(Canter.yRF)-hMax;
Canter.xLF = Canter.xLF - lStanceCanter/2;
Canter.yLF = -(Canter.yLF)-hMax;
Canter.xRR = Canter.xRR - lStanceCanter/2;
Canter.yRR = -(Canter.yRR)-hMax;
Canter.xLR = Canter.xLR - lStanceCanter/2;
Canter.yLR = -(Canter.yLR)-hMax;

% The next thing that needs to be adjusted is that right now we assume
% (0,0) is the position of the joint2 - if this is true, then (0,0) is
% a collision and also a singulatity which cannot give a solution for.
% In reality, the joint2 position should be (0,0) and the max swing height
% should be less than this.
% This ensures that the max y distance from the joint is the standing height!
Walk.yLF = Walk.yLF - (joint_height_from_ground - hMax);
Walk.yRF = Walk.yRF - (joint_height_from_ground - hMax);
Walk.yLR = Walk.yLR - (joint_height_from_ground - hMax);
Walk.yRR = Walk.yRR - (joint_height_from_ground - hMax);

Trot.yLF = Trot.yLF - (joint_height_from_ground - hMax);
Trot.yRF = Trot.yRF - (joint_height_from_ground - hMax);
Trot.yLR = Trot.yLR - (joint_height_from_ground - hMax);
Trot.yRR = Trot.yRR - (joint_height_from_ground - hMax);

Gallop.yLF = Gallop.yLF - (joint_height_from_ground - hMax);
Gallop.yRF = Gallop.yRF - (joint_height_from_ground - hMax);
Gallop.yLR = Gallop.yLR - (joint_height_from_ground - hMax);
Gallop.yRR = Gallop.yRR - (joint_height_from_ground - hMax);

Canter.yLF = Canter.yLF - (joint_height_from_ground - hMax);
Canter.yRF = Canter.yRF - (joint_height_from_ground - hMax);
Canter.yLR = Canter.yLR - (joint_height_from_ground - hMax);
Canter.yRR = Canter.yRR - (joint_height_from_ground - hMax);
%% Plot the generated curves and check for arm angle limit compliance
% First, find the max and min reach of the arm given the joint angle limits
joint2_low = deg2rad(-38);
joint2_high = deg2rad(170);
joint3_low = deg2rad(-156);
joint3_high= deg2rad(-48);

max_reach = 0;
min_reach = 1;
num_steps = 50;
workspace_x = []; % reachable workspace
workspace_y = [];
th2_range = linspace(joint2_low, joint2_high, num_steps);
th3_range = linspace(joint3_low, joint3_high, num_steps);
for i=1:num_steps
    th2 = th2_range(i);
    for j=1:num_steps
        th3 = th3_range(i);
        x_reach = a2*cos(th2)+a3*cos(th2+th3);
        y_reach = a2*sin(th2)+a3*sin(th2+th3);
        reach = sqrt(x_reach^2+y_reach^2);
        if reach > max_reach
            max_reach = reach;
        end
        if reach < min_reach
            min_reach = reach;
        end

        workspace_x = [workspace_x; x_reach];
        workspace_y = [workspace_y; y_reach];
    end
end
% Now plot the computed trajectories in x/y dimensions
figure();
scatter(Walk.xLF, Walk.yLF, 'DisplayName', 'Walk');hold on;
scatter(Trot.xLF, Trot.yLF, 'DisplayName', 'Trot');hold on;
scatter(Gallop.xLF, Gallop.yLF, 'DisplayName', 'Gallop');hold on;
scatter(Canter.xLF, Canter.yLF,'DisplayName','Canter');hold on;

% plot the robot arm for reference
[arm1_x, arm1_y] = pol2cart(th2Max, a2);
arm1_xcoords = linspace(0, arm1_x, 10);
arm1_ycoords = linspace(0, arm1_y, 10);
plot(arm1_xcoords, arm1_ycoords, 'LineWidth', 3, 'DisplayName', 'Thigh Link')
[arm2_dx, arm2_dy] = pol2cart(th2Max+th3Max, a3);
arm2_end_x = arm1_x + arm2_dx;
arm2_end_y = arm1_y + arm2_dy;
arm2_xcoords = linspace(arm1_x, arm2_end_x, 10);
arm2_ycoords = linspace(arm1_y, arm2_end_y, 10);
plot(arm2_xcoords, arm2_ycoords, 'LineWidth', 3, 'Color', 'blue', 'DisplayName', 'Calf Link')

% % plot a circle arc for the max reach and min reach 
% theta_circle = joint2_low+pi:0.1:joint2_high+pi;
% r_circle = max_reach;
% x = r_circle*cos(theta_circle);
% y = r_circle*sin(theta_circle);
% plot(x,y, 'DisplayName', 'Max Reach', 'LineWidth', 2)
% theta_circle = (joint3_low):0.1:joint3_high;
% r_circle = min_reach;
% x = r_circle*cos(theta_circle);
% y = r_circle*sin(theta_circle);
% plot(x,y, 'DisplayName', 'Min Reach', 'LineWidth', 2)


% scatter plot the reachable workspace (y gets same adjustments as data)
plot(workspace_x, -workspace_y-hMax, 'DisplayName', 'Reachable Space');
legend('Location', 'best');

title("Generated Paths Viapoints Through Swing Phase (with Joint Angle Reach Limits)")

%% Plot all x/y trajectories against time
% steps = linspace(0, length(Gallop.xLF), length(Gallop.xLF));
% subplot(4,1,1);
% % plot3(Gallop.xLF, Gallop.yLF, steps)
% plot(steps, Gallop.yLF)
% title("LF");
% subplot(4,1,2);
% % plot3(Gallop.xRF, Gallop.yRF,steps)
% plot(steps, Gallop.yRF)
% title("RF");
% subplot(4,1,3);
% % plot3(Gallop.xLR, Gallop.yLR, steps)
% plot(steps, Gallop.yLR)
% title("LR");
% subplot(4,1,4);
% % plot3(Gallop.xRR, Gallop.yRR, steps)
% plot(steps, Gallop.yRR)
% title("RR");
figure();
steps = linspace(0, length(Gallop.xLF), length(Gallop.xLF));
subplot(4,1,1);
% plot3(Gallop.xLF, Gallop.yLF, steps)
plot(steps, Gallop.yLF)
title("LF");
subplot(4,1,2);
% plot3(Gallop.xRF, Gallop.yRF,steps)
plot(steps, Gallop.yRF)
title("RF");
subplot(4,1,3);
% plot3(Gallop.xLR, Gallop.yLR, steps)
plot(steps, Gallop.yLR)
title("LR");
subplot(4,1,4);
% plot3(Gallop.xRR, Gallop.yRR, steps)
plot(steps, Gallop.yRR)
title("RR");

%% Inverse Kinematics

for i = 1:length(beta)+1
    if i == 1
        xLF = Walk.xLF;
        yLF = Walk.yLF;
        dLF = (xLF.^2+yLF.^2-a2^2-a3^2)/(2*a2*a3);
        Walk.th3LF = atan2(sqrt(1-dLF.^2),dLF);
        thLF = atan2(yLF,xLF);
        alphaLF = atan2(a3*sin(Walk.th3LF),a2+a3*cos(Walk.th3LF));
        Walk.th2LF = thLF-alphaLF;
        xRF = Walk.xRF;
        yRF = Walk.yRF;
        dRF = (xRF.^2+yRF.^2-a2^2-a3^2)/(2*a2*a3);
        Walk.th3RF = atan2(sqrt(1-dRF.^2),dRF);
        thRF = atan2(yRF,xRF);
        alphaRF = atan2(a3*sin(Walk.th3RF),a2+a3*cos(Walk.th3RF));
        Walk.th2RF = thRF-alphaRF;
        xRR = Walk.xRR;
        yRR = Walk.yRR;
        dRR = (xRR.^2+yRR.^2-a2^2-a3^2)/(2*a2*a3);
        Walk.th3RR = atan2(sqrt(1-dRR.^2),dRR);
        thRR = atan2(yRR,xRR);
        alphaRR = atan2(a3*sin(Walk.th3RR),a2+a3*cos(Walk.th3RR));
        Walk.th2RR = thRR-alphaRR;
        xLR = Walk.xLR;
        yLR = Walk.yLR;
        dLR = (xLR.^2+yLR.^2-a2^2-a3^2)/(2*a2*a3);
        Walk.th3LR = atan2(sqrt(1-dLR.^2),dLR);
        thLR = atan2(yLR,xLR);
        alphaLR = atan2(a3*sin(Walk.th3LR),a2+a3*cos(Walk.th3LR));
        Walk.th2LR = thLR-alphaLR;
    elseif i == 2
         xLF = Trot.xLF;
        yLF = Trot.yLF;
        dLF = (xLF.^2+yLF.^2-a2^2-a3^2)/(2*a2*a3);
        Trot.th3LF = atan2(sqrt(1-dLF.^2),dLF);
        thLF = atan2(yLF,xLF);
        alphaLF = atan2(a3*sin(Trot.th3LF),a2+a3*cos(Trot.th3LF));
        Trot.th2LF = thLF-alphaLF;
        xRF = Trot.xRF;
        yRF = Trot.yRF;
        dRF = (xRF.^2+yRF.^2-a2^2-a3^2)/(2*a2*a3);
        Trot.th3RF = atan2(sqrt(1-dRF.^2),dRF);
        thRF = atan2(yRF,xRF);
        alphaRF = atan2(a3*sin(Trot.th3RF),a2+a3*cos(Trot.th3RF));
        Trot.th2RF = thRF-alphaRF;
        xRR = Trot.xRR;
        yRR = Trot.yRR;
        dRR = (xRR.^2+yRR.^2-a2^2-a3^2)/(2*a2*a3);
        Trot.th3RR = atan2(sqrt(1-dRR.^2),dRR);
        thRR = atan2(yRR,xRR);
        alphaRR = atan2(a3*sin(Trot.th3RR),a2+a3*cos(Trot.th3RR));
        Trot.th2RR = thRR-alphaRR;
        xLR = Trot.xLR;
        yLR = Trot.yLR;
        dLR = (xLR.^2+yLR.^2-a2^2-a3^2)/(2*a2*a3);
        Trot.th3LR = atan2(sqrt(1-dLR.^2),dLR);
        thLR = atan2(yLR,xLR);
        alphaLR = atan2(a3*sin(Trot.th3LR),a2+a3*cos(Trot.th3LR));
        Trot.th2LR = thLR-alphaLR;
    elseif i == 3
        xLF = Gallop.xLF;
        yLF = Gallop.yLF;
        dLF = (xLF.^2+yLF.^2-a2^2-a3^2)/(2*a2*a3);
        Gallop.th3LF = atan2(sqrt(1-dLF.^2),dLF);
        thLF = atan2(yLF,xLF);
        alphaLF = atan2(a3*sin(Gallop.th3LF),a2+a3*cos(Gallop.th3LF));
        Gallop.th2LF = thLF-alphaLF;
        xRF = Gallop.xRF;
        yRF = Gallop.yRF;
        dRF = (xRF.^2+yRF.^2-a2^2-a3^2)/(2*a2*a3);
        Gallop.th3RF = atan2(sqrt(1-dRF.^2),dRF);
        thRF = atan2(yRF,xRF);
        alphaRF = atan2(a3*sin(Gallop.th3RF),a2+a3*cos(Gallop.th3RF));
        Gallop.th2RF = thRF-alphaRF;
        xRR = Gallop.xRR;
        yRR = Gallop.yRR;
        dRR = (xRR.^2+yRR.^2-a2^2-a3^2)/(2*a2*a3);
        Gallop.th3RR = atan2(sqrt(1-dRR.^2),dRR);
        thRR = atan2(yRR,xRR);
        alphaRR = atan2(a3*sin(Gallop.th3RR),a2+a3*cos(Gallop.th3RR));
        Gallop.th2RR = thRR-alphaRR;
        xLR = Gallop.xLR;
        yLR = Gallop.yLR;
        dLR = (xLR.^2+yLR.^2-a2^2-a3^2)/(2*a2*a3);
        Gallop.th3LR = atan2(sqrt(1-dLR.^2),dLR);
        thLR = atan2(yLR,xLR);
        alphaLR = atan2(a3*sin(Gallop.th3LR),a2+a3*cos(Gallop.th3LR));
        Gallop.th2LR = thLR-alphaLR;
    else
        xLF = Canter.xLF;
        yLF = Canter.yLF;
        dLF = (xLF.^2+yLF.^2-a2^2-a3^2)/(2*a2*a3);
        Canter.th3LF = atan2(sqrt(1-dLF.^2),dLF);
        thLF = atan2(yLF,xLF);
        alphaLF = atan2(a3*sin(Canter.th3LF),a2+a3*cos(Canter.th3LF));
        Canter.th2LF = thLF-alphaLF;
        xRF = Canter.xRF;
        yRF = Canter.yRF;
        dRF = (xRF.^2+yRF.^2-a2^2-a3^2)/(2*a2*a3);
        Canter.th3RF = atan2(sqrt(1-dRF.^2),dRF);
        thRF = atan2(yRF,xRF);
        alphaRF = atan2(a3*sin(Canter.th3RF),a2+a3*cos(Canter.th3RF));
        Canter.th2RF = thRF-alphaRF;
        xRR = Canter.xRR;
        yRR = Canter.yRR;
        dRR = (xRR.^2+yRR.^2-a2^2-a3^2)/(2*a2*a3);
        Canter.th3RR = atan2(sqrt(1-dRR.^2),dRR);
        thRR = atan2(yRR,xRR);
        alphaRR = atan2(a3*sin(Canter.th3RR),a2+a3*cos(Canter.th3RR));
        Canter.th2RR = thRR-alphaRR;
        xLR = Canter.xLR;
        yLR = Canter.yLR;
        dLR = (xLR.^2+yLR.^2-a2^2-a3^2)/(2*a2*a3);
        Canter.th3LR = atan2(sqrt(1-dLR.^2),dLR);
        thLR = atan2(yLR,xLR);
        alphaLR = atan2(a3*sin(Canter.th3LR),a2+a3*cos(Canter.th3LR));
        Canter.th2LR = thLR-alphaLR;
    end
end

%% Plot curve before transform
% figure();
% walk_qs_lf = [Gallop.th2LF', Gallop.th3LF'];
% L1 = Link('revolute', 'd', 0, 'a', a2, 'alpha', 0);
% L2 = Link('revolute', 'd', 0, 'a', a3, 'alpha', 0);
% bot = SerialLink([L1 L2], 'name', 'LF');
% figure();
% bot.plot(walk_qs_lf, 'view', 'top', 'workspace', [-0.5, 0.5, -0.5, 0.5, -1, 1],...
%     'delay', 0.01, 'trail', {'r', 'LineWidth', 2}) 

%% Fix joints for joint limits
% Looking at the walk gait, around the point (xLF=0.0025, yLF = -0.3385) is
% when the leg is very close to the "stand" position where we know from
% Unitree's servo.cpp file that theta2 = 0.67, theta3 = -1.3
% However, we calculated our angles in a different coordinate frame so we 
% need to shift. We got for Walk LF at the same x,y point above that 
% theta2 = -2.21 and theta3 = 1.3.
% Also there is a mirror flip over the y-axis to get joint-position = 0,0.
% Therefore, our transformations to the Unitree reference frame are
% theta2 = -theta2 -2.21 + 0.67
% theta3 = -theta3
% More generally, we can write this
% theta2_in_unitree_frame = -theta2 + theta2_computed_stand_angle + theta2_unitree_stand_angle
% And theta3 is simply a shift by pi so its a simple sign flip as shown above

% Walk gait adjustments, then ensure not outside of joint limits still
Walk.th2LF = convert_theta2_to_unitree(Walk.xLF, Walk.yLF, Walk.th2LF, joint_height_from_ground);
Walk.th3LF = -Walk.th3LF;
Walk.th2RF = convert_theta2_to_unitree(Walk.xRF, Walk.yRF, Walk.th2RF, joint_height_from_ground);
Walk.th3RF = -Walk.th3RF;
Walk.th2LR = convert_theta2_to_unitree(Walk.xLR, Walk.yLR, Walk.th2LR, joint_height_from_ground);
Walk.th3LR = -Walk.th3LR;
Walk.th2RR = convert_theta2_to_unitree(Walk.xRR, Walk.yRR, Walk.th2RR, joint_height_from_ground);
Walk.th3RR = -Walk.th3RR;
if (any(Walk.th2LF < joint2_low) || any(Walk.th2LF > joint2_high))
    error("Walk: Violating Joint 2 Joint Angles! Max angle: %.3f, min angle: %.3f\n" + ...
        "Try making the lStance smaller by reducing the gaitPeriod.", ...
        max(rad2deg(Walk.th2LF)), min(rad2deg(Walk.t2LF)));
end
if (any(Walk.th3LF < joint3_low) || any(Walk.th3LF > joint3_high))
    error("Walk: Violating Joint 3 Joint Angles! Max angle: %.3f, min angle: %.3f\n" + ...
        "Try making the lStance smaller by reducing the gaitPeriod.", ...
        max(rad2deg(Walk.th3LF)), min(rad2deg(Walk.t3LF)));
end

% Trot gait adjustments, then ensure not outside of joint limits still
Trot.th2LF = convert_theta2_to_unitree(Trot.xLF, Trot.yLF, Trot.th2LF, joint_height_from_ground);
Trot.th3LF = -Trot.th3LF;
Trot.th2RF = convert_theta2_to_unitree(Trot.xRF, Trot.yRF, Trot.th2RF, joint_height_from_ground);
Trot.th3RF = -Trot.th3RF;
Trot.th2LR = convert_theta2_to_unitree(Trot.xLR, Trot.yLR, Trot.th2LR, joint_height_from_ground);
Trot.th3LR = -Trot.th3LR;
Trot.th2RR = convert_theta2_to_unitree(Trot.xRR, Trot.yRR, Trot.th2RR, joint_height_from_ground);
Trot.th3RR = -Trot.th3RR;
if (any(Trot.th2LF < joint2_low) || any(Trot.th2LF > joint2_high))
    error("Trot: Violating Joint 2 Joint Angles! Max angle: %.3f, min angle: %.3f\n" + ...
        "Try making the lStance smaller by reducing the gaitPeriod.", ...
        max(rad2deg(Trot.th2LF)), min(rad2deg(Trot.th2LF)));
end
if (any(Trot.th3LF < joint3_low) || any(Trot.th3LF > joint3_high))
    error("Trot: Violating Joint 3 Joint Angles! Max angle: %.3f, min angle: %.3f\n" + ...
        "Try making the lStance smaller by reducing the gaitPeriod.", ...
        max(rad2deg(Trot.th3LF)), min(rad2deg(Trot.th3LF)));
end

% Gallop gait adjustments, then ensure not outside of joint limits still
Gallop.th2LF = convert_theta2_to_unitree(Gallop.xLF, Gallop.yLF, Gallop.th2LF, joint_height_from_ground);
Gallop.th3LF = -Gallop.th3LF;
Gallop.th2RF = convert_theta2_to_unitree(Gallop.xRF, Gallop.yRF, Gallop.th2RF, joint_height_from_ground);
Gallop.th3RF = -Gallop.th3RF;
Gallop.th2LR = convert_theta2_to_unitree(Gallop.xLR, Gallop.yLR, Gallop.th2LR, joint_height_from_ground);
Gallop.th3LR = -Gallop.th3LR;
Gallop.th2RR = convert_theta2_to_unitree(Gallop.xRR, Gallop.yRR, Gallop.th2RR, joint_height_from_ground);
Gallop.th3RR = -Gallop.th3RR;
if (any(Gallop.th2LF < joint2_low) || any(Gallop.th2LF > joint2_high))
    error("Gallop: Violating Joint 2 Joint Angles! Max angle: %.3f, min angle: %.3f\n" + ...
        "Try making the lStance smaller by reducing the gaitPeriod.", ...
        max(rad2deg(Gallop.th2LF)), min(rad2deg(Gallop.th2LF)));
end
if (any(Gallop.th3LF < joint3_low) || any(Gallop.th3LF > joint3_high))
    error("Gallop: Violating Joint 3 Joint Angles! Max angle: %.3f, min angle: %.3f\n" + ...
        "Try making the lStance smaller by reducing the gaitPeriod.", ...
        max(rad2deg(Gallop.th3LF)), min(rad2deg(Gallop.th3LF)));
end

% Canter gait adjustments, then ensure not outside of joint limits still
Canter.th2LF = convert_theta2_to_unitree(Canter.xLF, Canter.yLF, Canter.th2LF, joint_height_from_ground);
Canter.th3LF = -Canter.th3LF;
Canter.th2RF = convert_theta2_to_unitree(Canter.xRF, Canter.yRF, Canter.th2RF, joint_height_from_ground);
Canter.th3RF = -Canter.th3RF;
Canter.th2LR = convert_theta2_to_unitree(Canter.xLR, Canter.yLR, Canter.th2LR, joint_height_from_ground);
Canter.th3LR = -Canter.th3LR;
Canter.th2RR = convert_theta2_to_unitree(Canter.xRR, Canter.yRR, Canter.th2RR, joint_height_from_ground);
Canter.th3RR = -Canter.th3RR;
if (any(Canter.th2LF < joint2_low) || any(Canter.th2LF > joint2_high))
    error("Canter: Violating Joint 2 Joint Angles! Max angle: %.3f, min angle: %.3f\n" + ...
        "Try making the lStance smaller by reducing the gaitPeriod.", ...
        max(rad2deg(Canter.th2LF)), min(rad2deg(Canter.th2LF)));
end
if (any(Canter.th3LF < joint3_low) || any(Canter.th3LF > joint3_high))
    error("Canter: Violating Joint 3 Joint Angles! Max angle: %.3f, min angle: %.3f\n" + ...
        "Try making the lStance smaller by reducing the gaitPeriod.", ...
        max(rad2deg(Canter.th3LF)), min(rad2deg(Canter.th3LF)));
end

%% Generate a robot animation for a gait
walk_qs_lf = [Gallop.th2RF', Gallop.th3RF'];
L1 = Link('revolute', 'd', 0, 'a', a2, 'alpha', 0);
L2 = Link('revolute', 'd', 0, 'a', a3, 'alpha', 0);
bot = SerialLink([L1 L2], 'name', 'LF');
figure();
bot.plot(walk_qs_lf, 'view', 'top', 'workspace', [-0.5, 0.5, -0.5, 0.5, -1, 1],...
    'delay', 0.01, 'trail', {'r', 'LineWidth', 2}, ...
    'movie', 'test_gait.mp4') 
% walk_qs_rf = [Walk.th2RF', Walk.th3RF'];
% L1 = Link('revolute', 'd', 0, 'a', a2, 'alpha', 0);
% L2 = Link('revolute', 'd', 0, 'a', a3, 'alpha', 0);
% bot2 = SerialLink([L1 L2], 'name', 'RF');
% figure();
% bot2.plot(walk_qs_rf, 'view', 'top', 'workspace', [-0.5, 0.5, -0.5, 0.5, -1, 1],...
%     'delay', 0.01, 'trail', {'r', 'LineWidth', 2}, ...
%     'movie', 'walk_gait_rf.mp4')

% canter_qs_lf = [Canter.th2LF', Canter.th3LF'];
% L1 = Link('revolute', 'd', 0, 'a', a2, 'alpha', 0);
% L2 = Link('revolute', 'd', 0, 'a', a3, 'alpha', 0);
% bot3 = SerialLink([L1 L2], 'name', 'RF');
% figure();
% bot3.plot(canter_qs_lf, 'view', 'top', 'workspace', [-0.5, 0.5, -0.5, 0.5, -1, 1],...
%     'delay', 0.01, 'trail', {'r', 'LineWidth', 2}, ...
%     'movie', 'canter_gait_lf.mp4')
%% Create arrays for C++
% When we generated the joint angles, we made many small steps so that
% the curves were very smooth 
% Now we want to cut that down to be less smooth, but less commands to send
% to the robot in C++
desired_num_q = 24;

% Walk 
% Setup matrices for the thetas
total_q_walk = length(Walk.th2LF);
walk_q_lf = [Walk.th2LF', Walk.th3LF'];
walk_q_rf = [Walk.th2RF', Walk.th3RF'];
walk_q_lr = [Walk.th2LR', Walk.th3LR'];
walk_q_rr = [Walk.th2RR', Walk.th3RR'];
% Now select on the desired number of q's from our very smooth set
idx = round(linspace(1, total_q_walk, desired_num_q));
walk_q_lf = walk_q_lf(idx, :); 
walk_q_lf(end+1,:) = walk_q_lf(1,:);
walk_q_rf = walk_q_rf(idx, :); 
walk_q_rf(end+1,:) = walk_q_rf(1,:);
walk_q_lr = walk_q_lr(idx, :); 
walk_q_lr(end+1,:) = walk_q_lr(1,:);
walk_q_rr = walk_q_rr(idx, :); 
walk_q_rr(end+1,:) = walk_q_rr(1,:);
walk_q_rr(end+1, :) = walk_q_rr(1, :); 

% Trot 
% Setup matrices for the thetas
total_q_trot = length(Trot.th2LF);
trot_q_lf = [Trot.th2LF', Trot.th3LF'];
trot_q_rf = [Trot.th2RF', Trot.th3RF'];
trot_q_lr = [Trot.th2LR', Trot.th3LR'];
trot_q_rr = [Trot.th2RR', Trot.th3RR'];
% Now select on the desired number of q's from our very smooth set
idx = round(linspace(1, total_q_trot, desired_num_q));
trot_q_lf = trot_q_lf(idx, :); 
trot_q_lf(end+1,:) = trot_q_lf(1,:);
trot_q_rf = trot_q_rf(idx, :); 
trot_q_rf(end+1,:) = trot_q_rf(1,:);
trot_q_lr = trot_q_lr(idx, :); 
trot_q_lr(end+1,:) = trot_q_lr(1,:);
trot_q_rr = trot_q_rr(idx, :); 
trot_q_rr(end+1,:) = trot_q_rr(1,:);
trot_q_rr(end+1, :) = trot_q_rr(1, :); 

% Gallop 
% Setup matrices for the thetas
total_q_gallop = length(Gallop.th2LF);
gallop_q_lf = [Gallop.th2LF', Gallop.th3LF'];
gallop_q_rf = [Gallop.th2RF', Gallop.th3RF'];
gallop_q_lr = [Gallop.th2LR', Gallop.th3LR'];
gallop_q_rr = [Gallop.th2RR', Gallop.th3RR'];
% Now select on the desired number of q's from our very smooth set
idx = round(linspace(1, total_q_gallop, desired_num_q));
gallop_q_lf = gallop_q_lf(idx, :); 
gallop_q_lf(end+1,:) = gallop_q_lf(1,:);
gallop_q_rf = gallop_q_rf(idx, :); 
gallop_q_rf(end+1,:) = gallop_q_rf(1,:);
gallop_q_lr = gallop_q_lr(idx, :); 
gallop_q_lr(end+1,:) = gallop_q_lr(1,:);
gallop_q_rr = gallop_q_rr(idx, :); 
gallop_q_rr(end+1,:) = gallop_q_rr(1,:);
gallop_q_rr(end+1, :) = gallop_q_rr(1, :);  

% Canter 
% Setup matrices for the thetas
total_q_canter = length(Canter.th2LF);
canter_q_lf = [Canter.th2LF', Canter.th3LF'];
canter_q_rf = [Canter.th2RF', Canter.th3RF'];
canter_q_lr = [Canter.th2LR', Canter.th3LR'];
canter_q_rr = [Canter.th2RR', Canter.th3RR'];
% Now select on the desired number of q's from our very smooth set
idx = round(linspace(1, total_q_canter, desired_num_q));
canter_q_lf = canter_q_lf(idx, :); 
canter_q_lf(end+1,:) = canter_q_lf(1,:);
canter_q_rf = canter_q_rf(idx, :); 
canter_q_rf(end+1,:) = canter_q_rf(1,:);
canter_q_lr = canter_q_lr(idx, :); 
canter_q_lr(end+1,:) = canter_q_lr(1,:);
canter_q_rr = canter_q_rr(idx, :); 
canter_q_rr(end+1,:) = canter_q_rr(1,:);
canter_q_rr(end+1, :) = canter_q_rr(1, :);  

%% Format them as structs for C++
% This way we can copy and paste them simply into the C++ file
fprintf("struct jointAngles {\n");
fprintf("\tfloat q1_lf[%d]; // left front\n", desired_num_q);
fprintf("\tfloat q1_rf[%d]; // right front\n", desired_num_q);
fprintf("\tfloat q1_lr[%d]; // left rear\n", desired_num_q);
fprintf("\tfloat q1_rr[%d]; // right rear;\n", desired_num_q);
fprintf("\tfloat q2_lf[%d];\n", desired_num_q);
fprintf("\tfloat q2_rf[%d];\n", desired_num_q);
fprintf("\tfloat q2_lr[%d];\n", desired_num_q);
fprintf("\tfloat q2_rr[%d];\n", desired_num_q);
fprintf("};\n");
fprintf("\n");

% Walk
fprintf("jointAngles Walk = {\n");
% q1
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", walk_q_lf(i, 1))
end
fprintf("%.3f}, // q1_lf\n", walk_q_lf(end, 1)) % add on the last idx with }
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", walk_q_rf(i, 1));
end
fprintf("%.3f}, // q1_rf\n", walk_q_rf(end, 1));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", walk_q_lr(i, 1));
end
fprintf("%.3f}, // q1_lr\n", walk_q_lr(end, 1));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", walk_q_rr(i, 1));
end
fprintf("%.3f}, // q1_rr\n", walk_q_rr(end, 1));
% q2
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", walk_q_lf(i, 2));
end
fprintf("%.3f}, // q2_lf\n", walk_q_lf(end, 2)) ;
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", walk_q_rf(i, 2));
end
fprintf("%.3f}, // q2_rf\n", walk_q_rf(end, 2));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", walk_q_lr(i, 2));
end
fprintf("%.3f}, // q2_lr\n", walk_q_lr(end, 2));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", walk_q_rr(i, 2));
end
fprintf("%.3f} // q2_rr\n", walk_q_rr(end, 2));
fprintf("};\n");

fprintf("\n");
% Trot
fprintf("jointAngles Trot = {\n");
% q1
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", trot_q_lf(i, 1))
end
fprintf("%.3f}, // q1_lf\n", trot_q_lf(end, 1)) % add on the last idx with }
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", trot_q_rf(i, 1));
end
fprintf("%.3f}, // q1_rf\n", trot_q_rf(end, 1));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", trot_q_lr(i, 1));
end
fprintf("%.3f}, // q1_lr\n", trot_q_lr(end, 1));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", trot_q_rr(i, 1));
end
fprintf("%.3f}, // q1_rr\n", trot_q_rr(end, 1));
% q2
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", trot_q_lf(i, 2));
end
fprintf("%.3f}, // q2_lf\n", trot_q_lf(end, 2)) ;
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", trot_q_rf(i, 2));
end
fprintf("%.3f}, // q2_rf\n", trot_q_rf(end, 2));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", trot_q_lr(i, 2));
end
fprintf("%.3f}, // q2_lr\n", trot_q_lr(end, 2));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", trot_q_rr(i, 2));
end
fprintf("%.3f} // q2_rr\n", trot_q_rr(end, 2));
fprintf("};\n");

fprintf("\n");
% Gallop
fprintf("jointAngles Gallop = {\n");
% q1
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", gallop_q_lf(i, 1))
end
fprintf("%.3f}, // q1_lf\n", gallop_q_lf(end, 1)) % add on the last idx with }
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", gallop_q_rf(i, 1));
end
fprintf("%.3f}, // q1_rf\n", gallop_q_rf(end, 1));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", gallop_q_lr(i, 1));
end
fprintf("%.3f}, // q1_lr\n", gallop_q_lr(end, 1));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", gallop_q_rr(i, 1));
end
fprintf("%.3f}, // q1_rr\n", gallop_q_rr(end, 1));
% q2
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", gallop_q_lf(i, 2));
end
fprintf("%.3f}, // q2_lf\n", gallop_q_lf(end, 2)) ;
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", gallop_q_rf(i, 2));
end
fprintf("%.3f}, // q2_rf\n", gallop_q_rf(end, 2));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", gallop_q_lr(i, 2));
end
fprintf("%.3f}, // q2_lr\n", gallop_q_lr(end, 2));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", gallop_q_rr(i, 2));
end
fprintf("%.3f} // q2_rr\n", gallop_q_rr(end, 2));
fprintf("};\n");

% Canter
fprintf("jointAngles Canter = {\n");
% q1
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", canter_q_lf(i, 1))
end
fprintf("%.3f}, // q1_lf\n", canter_q_lf(end, 1)) % add on the last idx with }
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", canter_q_rf(i, 1));
end
fprintf("%.3f}, // q1_rf\n", canter_q_rf(end, 1));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", canter_q_lr(i, 1));
end
fprintf("%.3f}, // q1_lr\n", canter_q_lr(end, 1));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", canter_q_rr(i, 1));
end
fprintf("%.3f}, // q1_rr\n", canter_q_rr(end, 1));
% q2
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", canter_q_lf(i, 2));
end
fprintf("%.3f}, // q2_lf\n", canter_q_lf(end, 2)) ;
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", canter_q_rf(i, 2));
end
fprintf("%.3f}, // q2_rf\n", canter_q_rf(end, 2));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", canter_q_lr(i, 2));
end
fprintf("%.3f}, // q2_lr\n", canter_q_lr(end, 2));
fprintf("\t{ ");
for i=1:desired_num_q-1
    fprintf("%.3f, ", canter_q_rr(i, 2));
end
fprintf("%.3f} // q2_rr\n", canter_q_rr(end, 2));
fprintf("};\n");

% Code help
fprintf("\n// Joint angles are sent in this order for each leg: [hip, thigh, calf]\n");
fprintf("// Joint angles are sent in those order of legs: [FR, FL, RR, RL]\n");
fprintf("// We don't want to move hip joints - set those to 0.0.\n");
fprintf("// To access a single q value: float q = Walk.q1_lf[0]\n");
fprintf("// A joint angle position command is sent as a 12-element array.\n");
fprintf("// q1 joint angles must be between %.3f and %.3f rads\n", joint2_low, joint2_high);
fprintf("// q2 joint angles must be between %.3f and %.3f rads\n", joint3_low, joint3_high);
%% Function
% function fixed_th2_angles = convert_theta2_to_unitree(x_pos, y_pos, th2_angles, ground_height)
% % First we need to find the IK-computed angles when the robot is in stand position
% % Stand position is as close to x=0 and y = -joint_height_from_ground.
% unitree_stand_theta2 = 0.67;
% s = sign(x_pos);
% zeroCrossingIndices = find(s(1:end-1) ~= s(2:end));
% % Check the edges for a zero crossing 
% if s(1) ~= s(end) 
%     zeroCrossingIndices(end+1) = length(s);
% end
% closest_stand_idx = -1;
% nearest_yVal_to_stand_pos = 1;
% for i=1:length(zeroCrossingIndices)
%     idx = zeroCrossingIndices(i);
%     yVal = y_pos(idx);
%     height_from_ground = abs(yVal + ground_height);
%     if height_from_ground < 0.02 && height_from_ground < nearest_yVal_to_stand_pos
%         nearest_yVal_to_stand_pos = height_from_ground;
%         closest_stand_idx = idx;
%     end
% end
% % Now that we have the stand idx, we can find the theta2 value for stand position
% theta2_computed_stand_angle = th2_angles(closest_stand_idx);
% % and now we can compute the angles in the untree frame
% fixed_th2_angles = -th2_angles + theta2_computed_stand_angle + unitree_stand_theta2;
% end

function fixed_th2 = convert_theta2_to_unitree(x_pos, y_pos, th2, ground_height)

unitree_stand_theta2 = 0.65;

% Define the stand reference position in your IK frame
x_target = 0;
y_target = -ground_height;

% Find the closest index to the stand foot location
for i = 1:length(x_pos)
    dist(i) = norm([x_pos(i) - x_target, y_pos(i) - y_target]);
end
[~, stand_idx] = min(dist);

% Get the IK angle at that stand position
theta2_computed_stand = th2(stand_idx);

% Convert to Unitree joint frame
fixed_th2 = -th2 + theta2_computed_stand + unitree_stand_theta2;

end

