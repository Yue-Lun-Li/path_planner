clear all;
%%
fix = readmatrix("20230306/20230306_route1_height_novetal_fix.csv");
% ee_building_llh = [22.99665875 120.222584889 98.211];
ee_building_llh = fix(1,7:9);
fix2 = readmatrix("20230217/sunrise-033950hole1-2-for-test_fix");
%%
llh2 = fix2(:,7:9);
for i = 1:size(llh2,1)
    enu2(i,:) = llh2enu(llh2(i,:), ee_building_llh);
end
llh = fix(:,7:9);
for i = 1:size(llh,1)
    enu(i,:) = llh2enu(llh(i,:), ee_building_llh);
end
enu = [enu; enu2];
%%
max_x = max(enu(:,1));
min_x = min(enu(:,1));
max_y = max(enu(:,2));
min_y = min(enu(:,2));
interval_x = max_x - min_x;
interval_y = max_y - min_y;
%%
figure(1)
[xq,yq] = meshgrid(min_x-0.05*interval_x:.5:max_x+0.05*interval_x, min_y-0.05*interval_y:.5:max_y+0.05*interval_y);
vq = griddata(enu(:,1),enu(:,2),enu(:,3),xq,yq,'natural');
mesh(xq,yq,vq)
hold on
% plot3(enu(:,1),enu(:,2),enu(:,3),'o')
xlim([min_x-0.05*interval_x max_x+0.05*interval_x]);
ylim([min_y-0.05*interval_y max_y+0.05*interval_y]);
zlim([-10 10]);
%%
% figure(2)
% hmap_above = pcolor(xq,yq,vq);
% hmap_above.EdgeColor = [.5 .5 .5] ; % cosmetic adjustment
% colorbar
% colormap jet
% shading interp