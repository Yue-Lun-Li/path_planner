clc;
clear all;

%%
data_1 = readmatrix("bag/hybrid_a_1_path_details.csv");
%%
data_2 = readmatrix("bag/hybrid_a_pr_1_path_details.csv");

%%
pitch = flip(data_1(:,2));
roll = flip(data_1(:,3));
height = flip(data_1(:,4));
x= flip(data_1(:,5));
y = flip(data_1(:,6));

%%
pitch_pr = flip(data_2(:,2));
roll_pr = flip(data_2(:,3));
height_pr = flip(data_2(:,4));
x_pr= flip(data_2(:,5));
y_pr = flip(data_2(:,6));

%%
h_r = 5.2209/255;

figure(1)
subplot(3, 1, 1);
plot(pitch * 180 / pi);
hold on;
plot(pitch_pr * 180 / pi);
title('pitch');
legend('pitch', 'pitch_pr');
hold off;
ylabel('degree')
subplot(3, 1, 2);
plot(roll * 180 / pi);
hold on;
plot(roll_pr * 180 / pi);
title('roll');
legend('roll', 'roll_pr');
hold off;
ylabel('degree')
subplot(3, 1, 3);
plot(height * h_r);
hold on;
plot(height_pr * h_r);
title('height');
legend('height', 'height_pr');
ylabel('m')
hold off;


%%
% 指定 PNG 檔案的路徑
file_path = './simulation_2.png';

% 使用 imread 函數讀取 PNG 檔案
image_data = imread(file_path);
image_height = size(image_data,1);
% 可以將讀取的圖像顯示出來
figure(2);
imshow(image_data);
%%
t_y = -y + image_height;
t_y_pr = -y_pr + image_height;


%%
% 假設起始點和終點的坐標是 (start_x, start_y) 和 (end_x, end_y)
start_x = x(1);
start_y = t_y(1);
end_x = x(end);
end_y = t_y(end);

% 繪製起始點和終點的實心圓
hold on;
plot(x, t_y, 'r', 'LineWidth', 2);
plot(x_pr, t_y_pr, 'b', 'LineWidth', 2);
scatter(start_x, start_y, 'filled', 'MarkerFaceColor', 'g');  % 在起始點繪製綠色實心圓
scatter(end_x, end_y, 'filled', 'MarkerFaceColor', 'm');  % 在終點繪製品紅色實心圓
title('grid map');
legend('regular hybird A*', '', 'start', 'goal');  % 添加起始點和終點的圖例
hold off;
