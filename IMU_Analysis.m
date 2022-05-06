clc;clear;
fu='D:\LAB\新生作業\IMU\MT_0120053C-007-0002_00B419F5.csv';
ff='D:\LAB\新生作業\IMU\MT_0120053C-007-0002_00B41A0D.csv';
uq = xlsread(fu,'R6:U1287');
fq = xlsread(ff,'R6:U1287');
%% Rotation Matrix X Vector

%Upper Arm Vector
uxv = [];
for i=1:length(uq)
    quat = uq(i,:);
    rotm = quat2rotm(quat);
    uxv(:,i) = rotm(:,1);
end
Up_xvector = uxv.'; 

%Forearm Vector
fxv = [];
for i=1:length(fq)
    quat = fq(i,:);
    rotm = quat2rotm(quat);
    fxv(:,i) = rotm(:,1);
end
Fore_xvector = fxv.'; 
%% Arm Vector

su = Up_xvector.*Up_xvector;
sf = Fore_xvector.*Fore_xvector;
angle = [];
for i=1:length(Up_xvector)
    umag = sqrt(sum(su(i,:)));
    fmag = sqrt(sum(sf(i,:)));
    d = dot(Up_xvector(i,:),Fore_xvector(i,:));
    angle(i,1) = i; %frame
    angle(i,2) = rad2deg(acos(d/(umag*fmag))); %rad2deg: convert radius to degree
end
%% Plot Flexion Angle

x = angle(:,1)/100; %time
y = angle(:,2); %angle
plot(x,y);
title('Flexion Angle');
xlabel('Time(s)');
ylabel('Angle(°)');
%% 3D Animation

%Up_xvector = Up_xvector*30;
%Fore_xvector = Fore_xvector*25;
%for i = 1:length(Up_xvector)
%    uu = [0,Up_xvector(i,1)];
%    uv = [0,Up_xvector(i,2)];
%    uw = [0,Up_xvector(i,3)];
%    fu = [Up_xvector(i,1),Up_xvector(i,1)+Fore_xvector(i,1)];
%    fv = [Up_xvector(i,2),Up_xvector(i,2)+Fore_xvector(i,2)];
%    fw = [Up_xvector(i,3),Up_xvector(i,3)+Fore_xvector(i,3)];
%    plot3(uu,uv,uw,'r',fu,fv,fw,'b','linewidth',2);
%    grid on
%    grid minor
%    title('Full Arm Movement of Shooting Basketball');
%    xlabel('X(cm)');
%    ylabel('Y(cm)');
%    zlabel('Z(cm)');
%    axis([-60 60 -60 60 -60 60]);
%    pause(0.01);
%   F(i) = getframe(gcf);
%end
%video = VideoWriter('shooting.avi','Uncompressed AVI');
%open(video)
%writeVideo(video,F);
%close(video)