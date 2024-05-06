clear all;

%% set up data

fix = readmatrix("csv/h12_ob_map-novatel-fix");
image_name = "route12_resolution05_rtk.png";

%%
ee_building_llh = [22.99665875 120.222584889 98.211];
llh = fix(:,8:10);
for i = 1:size(llh,1)
    enu(i,:) = llh2enu(llh(i,:), ee_building_llh);
end
% enu(size(llh,1)+1,:) = enu(1,:);
%%
max_x = max(enu(:,1));
min_x = min(enu(:,1));
max_y = max(enu(:,2));
min_y = min(enu(:,2));
interval_x = max_x - min_x;
interval_y = max_y - min_y;
%%
plot(enu(:,1),enu(:,2));
%%
pgon = polyshape(enu(:,1),enu(:,2));
plot(pgon)
xlim([min_x-0.05*interval_x max_x+0.05*interval_x]);
ylim([min_y-0.05*interval_y max_y+0.05*interval_y]);
%%


%%
llh = fix(:,7:9);
for i = 1:size(llh,1)
    enu(i,:) = llh2enu(llh(i,:), ee_building_llh);
end



%%
% hold on
% fix = readmatrix("sunrise-033950hole1-1_fix");
% llh = fix(:,7:9);
% for i = 1:size(llh,1)
%     enu(i,:) = llh2enu(llh(i,:), ee_building_llh);
% end
% max_x = max(enu(:,1));
% min_x = min(enu(:,1));
% max_y = max(enu(:,2));
% min_y = min(enu(:,2));
% interval_x = max_x - min_x;
% interval_y = max_y - min_y;
% plot(enu(:,1),enu(:,2));



%% map generation
edge_len = 0.5; %0.5m*0.5m per pixel 
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



