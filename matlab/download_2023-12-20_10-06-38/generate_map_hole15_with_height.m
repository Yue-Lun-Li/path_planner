clear all;
%% load data
fix_edge = readmatrix("20230309/csv/sunrise-20230309165125_L1G3_hole15_frount_region_inspva.csv");
fix2_edge = readmatrix("20230309/csv/sunrise-20230309165125_L1G3_hole15_back_region_inspva.csv");
fix_inside = readmatrix("20230309/csv/sunrise-20230309165125_L1G3_hole15_frount_heigh_record_inspva.csv");
fix2_inside = readmatrix("20230309/csv/sunrise-20230309165125_L1G3_hole15_back_heigh_records_inspva.csv");
fix_bridge1 = readmatrix("20230309/csv/sunrise-20230309165125_L1G3_hole15_bidge_1_inspva.csv");
fix_bridge2 = readmatrix("20230309/csv/sunrise-20230309165125_L1G3_hole15_bridge_2_inspva.csv");
%% Transform coordinate to ENU
ee_building_llh = [22.99665875 120.222584889 98.211];
llh_edge = fix_edge(:,4:6);
for i = 1:size(llh_edge,1)
    enu_edge(i,:) = llh2enu(llh_edge(i,:), ee_building_llh);
end
llh2_edge = fix2_edge(:,4:6);
for i = 1:size(llh2_edge,1)
    enu2_edge(i,:) = llh2enu(llh2_edge(i,:), ee_building_llh);
end
llh_inside = fix_inside(:,4:6);
for i = 1:size(llh_inside,1)
    enu_inside(i,:) = llh2enu(llh_inside(i,:), ee_building_llh);
end
llh2_inside = fix2_inside(:,4:6);
for i = 1:size(llh2_inside,1)
    enu2_inside(i,:) = llh2enu(llh2_inside(i,:), ee_building_llh);
end

enu = [enu_edge; enu2_edge; enu_inside; enu2_inside];

llh_bridge1 = fix_bridge1(:,4:6);
for i = 1:size(llh_bridge1,1)
    enu_bridge1(i,:) = llh2enu(llh_bridge1(i,:), ee_building_llh);
end

llh_bridge2 = fix_bridge2(:,4:6);
for i = 1:size(llh_bridge2,1)
    enu_bridge2(i,:) = llh2enu(llh_bridge2(i,:), ee_building_llh);
end
%% Front region
% edge
for i = 1:size(enu_edge,1)
    if(enu_edge(i,1)>94731)
        start = i;
        break;
    end
end
enu_edge = enu_edge(start:end,:);

% inside
for i = 1:size(enu_inside,1)
    if(enu_inside(i,1)>94754)
        start = i;
        break;
    end
end
enu_inside = enu_inside(start:end,:);

enu1 = [enu_edge; enu_inside];
max_x = max(enu1(:,1));
min_x = min(enu1(:,1));
max_y = max(enu1(:,2));
min_y = min(enu1(:,2));
interval_x = max_x - min_x;
interval_y = max_y - min_y;

figure(1)
[xq,yq] = meshgrid(min_x-0.05*interval_x:.5:max_x+0.05*interval_x, min_y-0.05*interval_y:.5:max_y+0.05*interval_y);
vq = griddata(enu1(:,1),enu1(:,2),enu1(:,3),xq,yq,'natural');

pgon1 = polyshape(enu_edge(:,1),enu_edge(:,2));
for i = 1:size(vq,1)
    for j = 1:size(vq,2)
        pixel_center = [xq(i,j) yq(i,j)];
        if(~isinterior(pgon1,pixel_center(1),pixel_center(2)))
            vq(i,j) = NaN;
        end
    end
end

mesh(xq,yq,vq)
hold on
plot3(enu1(:,1),enu1(:,2),enu1(:,3),'o');
%% Plot parameter
max_x = max(enu(:,1));
min_x = min(enu(:,1));
max_y = max(enu(:,2));
min_y = min(enu(:,2));
interval_x = max_x - min_x;
interval_y = max_y - min_y;
xlim([min_x-0.05*interval_x max_x+0.05*interval_x]);
ylim([min_y-0.05*interval_y max_y+0.05*interval_y]);
zlim([-3900 -3860]);

%% Back region
enu2 = [enu2_edge; enu2_inside];
max_x = max(enu2(:,1));
min_x = min(enu2(:,1));
max_y = max(enu2(:,2));
min_y = min(enu2(:,2));
interval_x = max_x - min_x;
interval_y = max_y - min_y;

[xq,yq] = meshgrid(min_x-0.05*interval_x:.5:max_x+0.05*interval_x, min_y-0.05*interval_y:.5:max_y+0.05*interval_y);
vq = griddata(enu2(:,1),enu2(:,2),enu2(:,3),xq,yq,'natural');

pgon2 = polyshape(enu2_edge(:,1),enu2_edge(:,2));
for i = 1:size(vq,1)
    for j = 1:size(vq,2)
        pixel_center = [xq(i,j) yq(i,j)];
        if(~isinterior(pgon2,pixel_center(1),pixel_center(2)))
            vq(i,j) = NaN;
        end
    end
end

mesh(xq,yq,vq);
hold on
plot3(enu2(:,1),enu2(:,2),enu2(:,3),'o')
%% Bridge region
% bridge1
n = 1;
tmp = [];
for i = 1:size(enu_bridge1,1)
    pixel_center = enu_bridge1(i,:);
    if(~isinterior(pgon2,pixel_center(1),pixel_center(2)) && ~isinterior(pgon1,pixel_center(1),pixel_center(2)))
        tmp(n,:) = pixel_center;
        n = n + 1;
    end
end
enu_bridge1 = tmp;

direction_vec = [enu_bridge1(size(enu_bridge1,1),1)-enu_bridge1(1,1) enu_bridge1(size(enu_bridge1,1),2)-enu_bridge1(1,2)];
normal_vec = [-direction_vec(2) direction_vec(1)]/norm(direction_vec)/2;
for i = 1:size(enu_bridge1,1)
    left_bridge1(i,:) = [enu_bridge1(i,1)+normal_vec(1) enu_bridge1(i,2)+normal_vec(2) enu_bridge1(i,3)];
    right_bridge1(i,:) = [enu_bridge1(size(enu_bridge1,1)-i+1,1)-normal_vec(1) enu_bridge1(size(enu_bridge1,1)-i+1,2)-normal_vec(2) enu_bridge1(size(enu_bridge1,1)-i+1,3)];
end
bridge1 = [right_bridge1;left_bridge1];

max_x = max(bridge1(:,1));
min_x = min(bridge1(:,1));
max_y = max(bridge1(:,2));
min_y = min(bridge1(:,2));
interval_x = max_x - min_x;
interval_y = max_y - min_y;

[xq,yq] = meshgrid(min_x-0.05*interval_x:.5:max_x+0.05*interval_x, min_y-0.05*interval_y:.5:max_y+0.05*interval_y);
vq = griddata(bridge1(:,1),bridge1(:,2),bridge1(:,3),xq,yq,'natural');
mesh(xq,yq,vq)
hold on
plot3(bridge1(:,1),bridge1(:,2),bridge1(:,3),'o')

%% bridge2
n = 1;
tmp = [];
for i = 1:size(enu_bridge2,1)
    pixel_center = enu_bridge2(i,:);
    if(~isinterior(pgon2,pixel_center(1),pixel_center(2)) && ~isinterior(pgon1,pixel_center(1),pixel_center(2)))
        tmp(n,:) = pixel_center;
        n = n + 1;
    end
end
enu_bridge2 = tmp;

direction_vec = [enu_bridge2(size(enu_bridge2,1),1)-enu_bridge2(1,1) enu_bridge2(size(enu_bridge2,1),2)-enu_bridge2(1,2)];
normal_vec = [-direction_vec(2) direction_vec(1)]/norm(direction_vec)/2;
for i = 1:size(enu_bridge2,1)
    left_bridge2(i,:) = [enu_bridge2(i,1)+normal_vec(1) enu_bridge2(i,2)+normal_vec(2) enu_bridge2(i,3)];
    right_bridge2(i,:) = [enu_bridge2(size(enu_bridge2,1)-i+1,1)-normal_vec(1) enu_bridge2(size(enu_bridge2,1)-i+1,2)-normal_vec(2) enu_bridge2(size(enu_bridge2,1)-i+1,3)];
end
bridge2 = [right_bridge2;left_bridge2];

max_x = max(bridge2(:,1));
min_x = min(bridge2(:,1));
max_y = max(bridge2(:,2));
min_y = min(bridge2(:,2));
interval_x = max_x - min_x;
interval_y = max_y - min_y;

[xq,yq] = meshgrid(min_x-0.05*interval_x:.5:max_x+0.05*interval_x, min_y-0.05*interval_y:.5:max_y+0.05*interval_y);
vq = griddata(bridge2(:,1),bridge2(:,2),bridge2(:,3),xq,yq,'natural');
mesh(xq,yq,vq)
plot3(bridge2(:,1),bridge2(:,2),bridge2(:,3),'o')


