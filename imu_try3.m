%作業繳交用，有把初始座標改成x方向，還沒給負責的學長確認
close all; clear; clc;

tb_up = xlsread('up_4D.xlsx');
tb_down = xlsread('down_4D.xlsx');

L = 1282;
rotm_up = cell(L, 1);
rotm_down = cell(L, 1);

for s = 1:L
    rotm_up{s} = quat2rotm(tb_up(s, :));
    rotm_down{s} = quat2rotm(tb_down(s, :));
end

p0_s = [0; 0; 0]; %肩膀原點
p0_j = [30; 0; 0]; %手軸起始點
p0_h = [25; 0; 0]; %手掌起始點

p_j = cell(L, 1);
p_h = cell(L, 1);
for s = 1:L
    p_j{s} = rotm_up{s}*p0_j;
    p_h{s} = rotm_down{s}*p0_h;
    p_h{s} = p_h{s} + p_j{s};
end

vect_up = zeros(L, 3);
vect_down = zeros(L, 3);
for s = 1:L
    vect_up(s, 1) = p_j{s, 1}(1, 1) - p0_s(1, 1);
    vect_up(s, 2) = p_j{s, 1}(2, 1) - p0_s(2, 1);
    vect_up(s, 3) = p_j{s, 1}(3, 1) - p0_s(3, 1);
    vect_down(s, 1) = p_h{s, 1}(1, 1) - p_j{s, 1}(1, 1);
    vect_down(s, 2) = p_h{s, 1}(2, 1) - p_j{s, 1}(2, 1);
    vect_down(s, 3) = p_h{s, 1}(3, 1) - p_j{s, 1}(3, 1);
end

angle = zeros(L, 1);
for s = 1:L
    angle(s, 1) = VvV(vect_up(s, :), vect_down(s, :));
end

time = zeros(L, 1);
for t = 1:L
    time(t, 1) = t/100;
end
%{
figure;
plot(time, angle);
title('Joint Angle');
xlabel('Time(s)');
ylabel('Angle(degree)');
legend('Joint Angle');
%}

X1 = zeros(L, 2);
X2 = zeros(L, 2);
Y1 = zeros(L, 2);
Y2 = zeros(L, 2);
Z1 = zeros(L, 2);
Z2 = zeros(L, 2);
for s = 1:L
    X1(s, :) = [p0_s(1, 1);   p_j{s, 1}(1, 1)];
    X2(s, :) = [p_j{s, 1}(1, 1);   p_h{s, 1}(1, 1)];
    Y1(s, :) = [p0_s(2, 1);   p_j{s, 1}(2, 1)];
    Y2(s, :) = [p_j{s, 1}(2, 1);   p_h{s, 1}(2, 1)];
    Z1(s, :) = [p0_s(3, 1);   p_j{s, 1}(3, 1)];
    Z2(s, :) = [p_j{s, 1}(3, 1);   p_h{s, 1}(3, 1)];
end

figure;
pause(5)
for s = 1:L
    plot3(X1(s, :), Y1(s, :), Z1(s, :), X2(s, :), Y2(s, :), Z2(s, :), 'linewidth', 2)
    axis ([-60 60 -60 60 -60 60])
    title('Shooting Basketball Simulation');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    grid on
    pause(0.001)
    M(s) = getframe(gcf);
end

avi = VideoWriter('imu.avi');
open(avi)
writeVideo(avi, M)
close(avi)

%影片儲存成avi檔
avifile = VideoWriter('vicon.avi');
open(avifile)
writeVideo(avifile, M)
close(avifile)

%}

function [ang] = VvV(vA, vB)
%vA=[x, y]
lenA = norm(vA);
lenB = norm(vB);
rad = acos(dot(vA,vB)/(lenA*lenB));
ang = rad*180/pi;
end