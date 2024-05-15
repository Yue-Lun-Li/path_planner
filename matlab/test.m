
clear all;
%%
%%novatel lla collection & to enu
% fix1 = readmatrix('csv/231225_YL/hole1_1-novatel-fix.csv');
% fix2 = readmatrix('csv/231225_YL/hole1_2-novatel-fix.csv');
% fix3 = readmatrix('csv/231225_YL/hole1_3-novatel-fix.csv');
% fix4 = readmatrix('csv/231225_YL/hole1_4-novatel-fix.csv');
% fix5 = readmatrix('csv/231225_YL/hole1_5-novatel-fix.csv');
% fix6 = readmatrix('csv/231225_YL/hole1_6-novatel-fix.csv');
% fix7 = readmatrix('csv/231225_YL/hole1_7-novatel-fix.csv');
% llh =  vertcat(fix1(:,8:10), fix2(:,8:10), fix3(:,8:10), fix4(:,8:10), fix5(:,8:10), fix6(:,8:10), fix7(:,8:10));

ee_building_llh = [22.99665875 120.222584889 98.211];

 
fix = readmatrix('csv/novatel_fix.csv');
llh = fix(:,8:10);
for i = 1:size(llh,1)
    enu(i,:) = llh2enu(llh(i,:), ee_building_llh);
end
%%
max_x = max(enu(:,1));
min_x = min(enu(:,1));
max_y = max(enu(:,2));
min_y = min(enu(:,2));

max_z = max(enu(:,3));
min_z = min(enu(:,3));


interval_x = max_x - min_x;
interval_y = max_y - min_y;
%%
[Xq, Yq] = meshgrid(linspace(min_x, max_x, 100), linspace(min_y, max_y, 100));

Zq = griddata(enu(:,1), enu(:,2), enu(:,3), Xq, Yq, 'linear');

%%
edge_len = 0.5; %1m*1m per pixel 
map_origin = [round(min_x-0.05*interval_x) round(min_y-0.05*interval_y)]
rows = round(1.1*interval_y/edge_len);
cols = round(1.1*interval_x/edge_len);
grid_map = zeros(rows,cols);

%%
for i = 1:rows
    for j = 1:cols
        pixel_center = map_origin + edge_len*[j-1 i-1];
            % 插值高度數據
        alt_value = interp2(Xq, Yq, Zq, pixel_center(1), pixel_center(2), 'cubic');
        altitude_map(i, j) = alt_value;
    end
end
%%
altitude_map_norm = altitude_map - min(altitude_map(:));  % 减去最小值使数据最小为0
altitude_map_norm = altitude_map_norm / max(altitude_map_norm(:));  % 除以最大值使数据最大为1
altitude_map_norm = uint8(255 * altitude_map_norm);  % 转换为0-255范围

%%
a = max(altitude_map(:))-min(altitude_map(:))

%%
figure;
plot(altitude_map_norm);
title('Normalized Altitude Map');

%%
imwrite(altitude_map_norm, colormap(gray(256)), 'grid_map_05.png');


%%
figure(1)
surf(Xq,Yq,Zq)
shading interp
colorbar
%%
% surf(xt,yt,zt)

plot3(enu(:,1),enu(:,2),enu(:,3),'o')
xlabel('e');
ylabel('n');
zlabel('u');
grid on
colorbar

