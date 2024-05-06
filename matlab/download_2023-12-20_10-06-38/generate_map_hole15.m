clear all;
clc; 
%% load inspva data
fix = readmatrix("20230309/csv/sunrise-20230309165125_L1G3_hole15_frount_region_inspva.csv");
fix2 = readmatrix("20230309/csv/sunrise-20230309165125_L1G3_hole15_back_region_inspva.csv");
fix_bridge1 = readmatrix("20230309/csv/sunrise-202205021730_L1G1_route_br1-ublox_f9k .csv");
fix_bridge2 = readmatrix("20230309/csv/sunrise-202205021730_L1G1_route_br2-ublox_f9k.csv");
ee_building_llh = [22.99665875 120.222584889 98.211];
%% read fix and transfer to enu related to ee_building_llh
llh = fix(:,4:6);
for i = 1:size(llh,1)
    enu1(i,:) = llh2enu(llh(i,:), ee_building_llh);
end

llh2 = fix2(:,4:6);
for i = 1:size(llh2,1)
    enu2(i,:) = llh2enu(llh2(i,:), ee_building_llh);
end

llh_bridge1 = fix_bridge1(:,21:23);
for i = 1:size(llh_bridge1,1)
    enu_bridge1(i,:) = llh2enu(llh_bridge1(i,:), ee_building_llh);
end

llh_bridge2 = fix_bridge2(:,21:23);
for i = 1:size(llh_bridge2,1)
    enu_bridge2(i,:) = llh2enu(llh_bridge2(i,:), ee_building_llh);
end

enu = [enu1; enu2];
%% Front region
for i = 1:size(enu1,1)
    if(enu1(i,1)>94731)
        start = i;
        break;
    end
end
pgon1 = polyshape(enu1(start:end,1),enu1(start:end,2));

%% Back region
pgon2 = polyshape(enu2(:,1),enu2(:,2));

%% Bridge region
plot(enu_bridge1(:,1),enu_bridge1(:,2))
direction_vec = [enu_bridge1(size(enu_bridge1,1),1)-enu_bridge1(1,1) enu_bridge1(size(enu_bridge1,1),2)-enu_bridge1(1,2)];
normal_vec = [-direction_vec(2) direction_vec(1)]/norm(direction_vec)/2;
for i = 1:size(enu_bridge1,1)
    left_bridge1(i,:) = [enu_bridge1(i,1)+normal_vec(1) enu_bridge1(i,2)+normal_vec(2)];
    right_bridge1(i,:) = [enu_bridge1(size(enu_bridge1,1)-i+1,1)-normal_vec(1) enu_bridge1(size(enu_bridge1,1)-i+1,2)-normal_vec(2)];
end
bridge1 = [right_bridge1;left_bridge1];
pgon_bridge1 = polyshape(bridge1(:,1),bridge1(:,2));


direction_vec = [enu_bridge2(size(enu_bridge2,1),1)-enu_bridge2(1,1) enu_bridge2(size(enu_bridge2,1),2)-enu_bridge2(1,2)];
normal_vec = [-direction_vec(2) direction_vec(1)]/norm(direction_vec)/2;
for i = 1:size(enu_bridge2,1)
    left_bridge2(i,:) = [enu_bridge2(i,1)+normal_vec(1) enu_bridge2(i,2)+normal_vec(2)];
    right_bridge2(i,:) = [enu_bridge2(size(enu_bridge2,1)-i+1,1)-normal_vec(1) enu_bridge2(size(enu_bridge2,1)-i+1,2)-normal_vec(2)];
end
bridge2 = [right_bridge2;left_bridge2];
pgon_bridge2 = polyshape(bridge2(:,1),bridge2(:,2));
% plot(pgon_bridge2)
% hold off

% %% show image
% figure(1)
% hold on
% plot(pgon1)
% plot(pgon_bridge1)
% plot(pgon2)
% plot(pgon_bridge2)
% hold off
%% map generation
max_x = max(enu(:,1));
min_x = min(enu(:,1));
max_y = max(enu(:,2));
min_y = min(enu(:,2));
interval_x = max_x - min_x;
interval_y = max_y - min_y;

edge_len = 0.5; %1m*1m per pixel 

map_origin = [round(min_x-0.05*interval_x) round(min_y-0.05*interval_y)]
rows = round(1.1*interval_y/edge_len);
cols = round(1.1*interval_x/edge_len);
grid_map = zeros(rows,cols);
image = zeros(rows,cols);
count = 0;
for i = 1:rows
    for j = 1:cols
        pixel_center = map_origin + edge_len*[j-1 i-1];
        if(isinterior(pgon1,pixel_center(1),pixel_center(2)) || isinterior(pgon2,pixel_center(1),pixel_center(2)) ...
                || isinterior(pgon_bridge1,pixel_center(1),pixel_center(2)) || isinterior(pgon_bridge2,pixel_center(1),pixel_center(2)))
            grid_map(i, j) = 1;
            image(rows+1-i, j) = 255;
            count = count+1
        end
    end
end

%%
% image = zeros(rows,cols);
% count = 0;
% for i = 1:rows
%     for j = 1:cols
%         if(grid_map(i, j) == 1)
%             image(rows+1-i, j) = 255;
%             count=count+1
%         end
%     end
% end
% figure(3)
% imshow(image)
%%
imwrite(image,"route15_resolution05_rtk.png");
