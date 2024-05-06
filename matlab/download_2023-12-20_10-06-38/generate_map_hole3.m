clear all;
image_name = "route3_resolution05_rtk_1001.png";
ee_building_llh = [22.99665875 120.222584889 98.211];
%% basic region
fix = readmatrix("20230710/sunrise-20230622090000_hole3_map-novatel-fix");
llh = fix(:,8:10);
for i = 1:size(llh,1)
    enu(i,:) = llh2enu(llh(i,:), ee_building_llh);
end

plot(enu(:,1),enu(:,2));
pgon = polyshape(enu(:,1),enu(:,2));

%% expand entrance region 
fix1 = readmatrix("20230919/hole3_1-novatel-fix");
llh1 = fix1(:,8:10);
for i = 1:size(llh1,1)
    enu1(i,:) = llh2enu(llh1(i,:), ee_building_llh);
end
plot(enu1(:,1),enu1(:,2));
pgon1 = polyshape(enu1(:,1),enu1(:,2));

%% expand left-side region 
fix2 = readmatrix("20230919/hole3_2-novatel-fix");
llh2 = fix2(:,8:10);
for i = 1:size(llh2,1)
    enu2(i,:) = llh2enu(llh2(i,:), ee_building_llh);
end
plot(enu2(:,1),enu2(:,2));
pgon2 = polyshape(enu2(:,1),enu2(:,2));

%%
max_x = max(enu(:,1));
min_x = min(enu(:,1));
max_y = max(enu(:,2));
min_y = min(enu(:,2));
max_x1 = max(enu1(:,1));
min_x1 = min(enu1(:,1));
max_y1 = max(enu1(:,2));
min_y1 = min(enu1(:,2));
max_x2 = max(enu2(:,1));
min_x2 = min(enu2(:,1));
max_y2 = max(enu2(:,2));
min_y2 = min(enu2(:,2));
if(max_x<max_x1)
    max_x = max_x1;
end
if(min_x>min_x1)
    min_x = min_x1;
end
if(max_y<max_y1)
    max_y = max_y1;
end
if(min_y>min_y1)
    min_y = min_y1;
end
if(max_x<max_x2)
    max_x = max_x2;
end
if(min_x>min_x2)
    min_x = min_x2;
end
if(max_y<max_y2)
    max_y = max_y2;
end
if(min_y>min_y2)
    min_y = min_y2;
end


%% find interval and limit
interval_x = max_x - min_x;
interval_y = max_y - min_y;
xlim([min_x-0.05*interval_x max_x+0.05*interval_x]);
ylim([min_y-0.05*interval_y max_y+0.05*interval_y]);


%% map generation
edge_len = 0.5; %1m*1m per pixel 
map_origin = [round(min_x-0.05*interval_x) round(min_y-0.05*interval_y)]
rows = round(1.1*interval_y/edge_len);
cols = round(1.1*interval_x/edge_len);
grid_map = zeros(rows,cols);
for i = 1:rows
    for j = 1:cols
        pixel_center = map_origin + edge_len*[j-1 i-1];
        if(isinterior(pgon,pixel_center(1),pixel_center(2)))
            grid_map(i, j) = 1;
        end
         if(isinterior(pgon1,pixel_center(1),pixel_center(2)))
            grid_map(i, j) = 1;
         end
        if(isinterior(pgon2,pixel_center(1),pixel_center(2)))
            grid_map(i, j) = 1;
        end       
    end
end

%%
image = zeros(rows,cols);
for i = 1:rows
    for j = 1:cols
        if(grid_map(i, j) == 1)
            image(rows+1-i, j) = 255;
        end
    end
end
figure(3)
imshow(image)
%%
imwrite(image,image_name);
