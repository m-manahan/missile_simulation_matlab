clear all; close all; clc; format long;
name = 'Marcus Manahan';
hw_num = 'project';
%Create arrays to hold the X,Y,Z coordinates, inital mass, final mass, and
%other variables
X0 = zeros(1,7);
Y0= zeros(1,7);
Z0= zeros(1,7);
m0= zeros(1,7);
mf= zeros(1,7);
Thmag0= zeros(1,7);
theta= zeros(1,7); 
phi= zeros(1,7);
Tburn= zeros(1,7);
%read in all of the values from missile_data.txt
 for n = 1:7
     
     [ X0(n), Y0(n), Z0(n), m0(n), mf(n), Thmag0(n), theta(n), phi(n), Tburn(n) ] = read_input('missile_data.txt',n);
 end
    Tval = cell(1,7);
    Xval = cell(1,7);
    Yval = cell(1,7);
    Zval = cell(1,7);
    Uvel = cell(1,7);
    Vvel = cell(1,7);
    Wvel = cell(1,7);
%run simulation for all 7 missiles
 for m = 1:7
 [ Tval{m},Xval{m},Yval{m},Zval{m},Uvel{m},Vvel{m},Wvel{m}] = missile(X0(m),Y0(m),Z0(m),m0(m),mf(m),Thmag0(m),theta(m),phi(m),Tburn(m));
 end

%Plot of Trajectories and Landing Locations
load('terrain.mat');
figure(1); hold on;
surf(x_terrain/1000, y_terrain/1000, h_terrain/1000); 
shading interp; 
color = {'-r', '-k', '-c', '-y', '-g', '-b', '-m'};
color1 = {'ro', 'ko', 'co', 'yo', 'go', 'bo', 'mo'};
color2 = {'r', 'k', 'c', 'y', 'g', 'b', 'm'};
color3 = {'r*', 'k*', 'c*', 'y*', 'g*', 'b*', 'm*'};
%plot the trajectories
for g = 1:7
    plot3(Xval{g}./1000,Yval{g}./1000,Zval{g}./1000,color{g});
end
h_legend = legend('terrain','M.1','M.2','M.3','M.4','M.5','M.6','M.7');
set(h_legend,'FontSize',10);
%plot the landing locations
for k = 1:7
    plot3(Xval{k}(end)/1000, Yval{k}(end)/1000, Zval{k}(end)/1000,color1{k}, ...
        'MarkerSize', 10, 'MarkerFaceColor', color2{k}, 'MarkerEdgeColor', color2{k});
end
hold off;
xlabel('x (km)'); ylabel('y (km)'); zlabel('z (km)');
view(3); axis([0 30 0 30 0 3.5]); grid on; 
set(gca,'LineWidth',2,'FontSize',16, ...
        'Xtick',[0:5:30],'Ytick',[0:5:30],'Ztick',[0:.5:3.5]);
title('Trajectories and Landing Locations')

 figure(2); %hold on; %subplot(2,1,1);
for c = 1:7
    subplot(2,1,1);
    hold on;
    plot(Tval{c},(((Uvel{c}(1:end).^2+Vvel{c}(1:end).^2+Wvel{c}(1:end).^2)).^(1/2))./340,color2{c});
end
title('Evolution of speed and accleration'); %%placed the title for the first subplot here
ylabel('Ma');
%acc/g versus time plot
Accvec = cell(1,7);
magV = cell(1,7);
for h =1:7
len1 = numel(Uvel{h});
Vmagpart = zeros(1,len1-1);
Vmagvec = (Uvel{h}(1:end).^2+Vvel{h}(1:end).^2+Wvel{h}(1:end).^2).^(1/2);
    for f = 2:len1
        Vmagpart(f-1) = Vmagvec(f) - Vmagvec(f-1);  
    end
    Accvec{h} = Vmagpart./(.005);
    magV{h} = Vmagvec;
end
%put labels on the axis
 for r = 1:7
 subplot(2,1,2);
 hold on;
 plot(Tval{r}(2:end),Accvec{r}(1:end)./9.81,color2{r});
 end
 h_legend = legend('M.1','M.2','M.3','M.4','M.5','M.6','M.7');
set(h_legend,'FontSize',10);
xlabel('Time(s)');
ylabel('Acc/g');
hold off;
figure(3); hold on;
%plot Mach number versus Altitude in km
Mach = cell(1,7);
for y = 1:7
    Mach{y} = magV{y}./340;
end
for q = 1:7
    %plot(magV{q}./340,Zval{q}./1000,color2{q});
    plot(Mach{q},Zval{q}./1000,color2{q});
end
xlim([0 9]);
title('Sonic Barrier');
xlabel('Ma');
ylabel('Altitude (km)');
h_legend = legend('M.1','M.2','M.3','M.4','M.5','M.6','M.7');
set(h_legend,'FontSize',10);

for e = 1:7
    len1 = numel(magV{e});
    for n = 2:len1-1
        if ((Mach{e}(n) > 1)&& (Mach{e}(n+1)<1)) || (Mach{e}(n)>1 && Mach{e}(n-1)<1)
            plot(Mach{e}(n),Zval{e}(n)/1000,color3{e});
        end
    end
% plot(xval should be 1,,color3{e});
end
hold off;
landTimes = zeros(1,7);
land_loc = cell(1,7);
lanMa = zeros(1,7);
lanAcc = zeros(1,7);
distance = cell(1,7);
max_heightpos = cell(1,7);
max_Ma = cell(1,7);
maxhAcc = cell(1,7);
for u = 1:7
    %process all the information for the flight stat struct
    landTimes(u) = Tval{u}(end); 
    land_loc{u} = [Xval{u}(end) Yval{u}(end) Zval{u}(end)];
    lanMa(u) = Mach{u}(end);
    lanAcc(u) = Accvec{u}(end);
    %%compute the travel distance
     delX = diff(Xval{u});
     delY = diff(Yval{u});
     delZ = diff(Zval{u});
   distance{u} = sum((delX.^2+delY.^2+delZ.^2).^(1/2));
   %%compute x y z position at the maximum altitude
   [Zmaxval,indmaxval] = max(Zval{u});
    Ymaxval = Yval{u}(indmaxval);
    Xmaxval = Xval{u}(indmaxval);
    max_heightpos{u} = [Xmaxval Ymaxval Zmaxval];
    %%compute Mach number at the maximum altitude
    max_Ma{u} = Mach{u}(indmaxval);
    %%Acceleration at max height
    maxhAcc{u} = Accvec{u}(indmaxval-1);
end
    flight_stat = struct('missile_ID',{},'landing_time',{},'travel_distance',{},'max_height_position',{},'max_height_Ma',{},...
        'max_height_Acc',{},'landing_location',{},'landing_Ma',{},'landing_Acc',{});
    for o = 1:7
        flight_stat(o).missile_ID = o;
        flight_stat(o).landing_time = landTimes(o);
        flight_stat(o).travel_distance = distance{o}; 
        flight_stat(o).max_height_position =  max_heightpos{o}; 
        flight_stat(o).max_height_Ma = max_Ma{o};
        flight_stat(o).max_height_Acc = maxhAcc{o}; 
        flight_stat(o).landing_location = land_loc{o};
        flight_stat(o).landing_Ma = lanMa(o);
        flight_stat(o).landing_Acc = lanAcc(o);
    end
fileID = fopen('report.txt','w');
fprintf(fileID,'%s\n%s\n%s\n','Marcus Manahan','A10980948','M_ID, landing time (s), travel distance (m), landing speed (m/s), landing acceleration (m/s^2)');
for b = 1:7
    fprintf(fileID,'%d %15.9e %15.9e %15.9e %15.9e\n', flight_stat(b).missile_ID, flight_stat(b).landing_time, flight_stat(b).travel_distance, magV{b}(end), flight_stat(b).landing_Acc);
end
fclose(fileID);
    p1a = 'See figure 1';
    p1b = 'See figure 2';
    p1c = 'See figure 3';
    p2a = flight_stat(1);
    p2b = flight_stat(2);
    p2c = flight_stat(3);
    p2d = flight_stat(4);
    p2e = flight_stat(5);
    p2f = flight_stat(6);
    p2g = flight_stat(7);
    p3 = evalc('type report.txt');

    %
    %Movie Part
% Load missile trajectories and terrain:
% missile_results.mat contains 7 cell arrays: T, X, Y, Z, U, V, and W.
% terrain.mat contains terrain geometry: x_terrain, y_terrain, h_terrain.
%load('missile_results.mat');
load('terrain.mat');

% Padding so that all vectors have the same length.
    max_length = 0;
i=numel(Tval);
for n = 1:i
    if max_length < numel(Tval{n})
        max_length = numel(Tval{n});
    end
end

% Use another 50 elements for padding
max_length = max_length+50;
for n = 1:i
    Tval{n}(end+1:max_length) = Tval{n}(end);
    Uvel{n}(end+1:max_length) = Uvel{n}(end);
    Vvel{n}(end+1:max_length) = Vvel{n}(end);
    Wvel{n}(end+1:max_length) = Wvel{n}(end);
    Xval{n}(end+1:max_length) = Xval{n}(end);
    Yval{n}(end+1:max_length) = Yval{n}(end);
    Zval{n}(end+1:max_length) = Zval{n}(end);
end

% Initiate video object:
vidObj = VideoWriter('missile_validation.avi');
vidObj.Quality = 100;
vidObj.FrameRate = 10;
open(vidObj);
figure('position',[100 100 1000 600],'visible','on');

%Create figure:
step = 50; % stride: smaller value makes video file larger.
fsize = 16; 
cs = 'krbgmcy';
     
for k = 1:step:max_length
    for id=1:7
        % Plot terrain surface
        s =surf(x_terrain/1000, y_terrain/1000, h_terrain/1000);
        hold on;
        % Plot trajectory and end values        
        gh1{id} = plot3(Xval{id}(1:k)/1e3,Yval{id}(1:k)/1e3,Zval{id}(1:k)/1e3,cs(id)); 
        plot3(Xval{id}(k)/1e3,Yval{id}(k)/1e3,Zval{id}(k)/1e3,[cs(id) 'o'], ...
               'MarkerFacecolor',cs(id),'MarkerEdgecolor',cs(id), ...
               'MarkerSize',10);
        grid on; box on; colormap('summer'); caxis([0 1.5]);
        axis([0 30 0 30 0 3.5]);
        set(gca,'LineWidth',2,'FontSize',fsize, ...
           'Xtick',[0:5:30],'Ytick',[-0:5:30],'Ztick',[0:.5:3]);
        set(gca,'Position',[0.07 0.07 0.88 0.6]);   
            
        time_string = sprintf('Time: %05.3f s', Tval{1}(k));
        text(-25,0,10,time_string,'Fontsize',fsize,'Color','k');
        header_text = sprintf('%2s %12s %18s %18s %16s %16s %16s %20s',...
                          'ID','X','Y','Z','U','V','W','status');
        text(-13,0,9.5,header_text,'Fontsize',fsize,'Color','k');
        if k >= 2 
            if (Tval{id}(k) == Tval{id}(k-1))
                status_string = 'landed';
            else
                status_string = 'in-flight';
            end
        else
            status_string = 'in-flight';
        end
        
        if k >= max_length-step; status_string = 'landed'; end;
        
        data_string = sprintf('%2d    %09.6f    %09.6f    %08.6f   %08.6f   %09.6f    %09.6f      %s', ...
                           id,Xval{id}(k)/1e3,Yval{id}(k)/1e3,Zval{id}(k)/1e3,...
                               Uvel{id}(k)/1e3,Vvel{id}(k)/1e3,Wvel{id}(k)/1e3,status_string);
        text(-13,0,9.5-0.35*id,data_string,'Fontsize',16,'Color',cs(id));
        text(13,0,5.6,'(position in km, velocity in km/s)', ...
                        'FontSize',fsize-2);
        set(gh1{id},'LineWidth',2);         
    end
    xlabel('x (km)','Position',[10,-7,0]);
    ylabel('y (km)','Position',[-7,10,0]);
    zlabel('z (km)'); view(3);
    title('Missile Trajectories','Position',[7,5,8.75]);
    
    % Write frame to the video object:
    currFrame = getframe(gcf);
    writeVideo(vidObj,currFrame);
    hold off;
end

% Close video object:
close(vidObj);
    %