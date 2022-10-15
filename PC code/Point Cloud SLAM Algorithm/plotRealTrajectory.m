% Code for plotting real trajectory motion
% x1 = [0 0];
% y1 = [0 7.48];
% 
% x2 = [0 7.48];
% y2 = [7.48 7.48]; 
% 
% x3 = [6.76 6.76];
% y3 = [6.76 0];
% 
% plot(x1, y1, '-*r', 'LineWidth', 2);
% hold on; 
% plot(x2, y2, '-*r', 'LineWidth', 2);
% % hold on; 
% % plot(x3, y3, '-*r', 'LineWidth', 2);
% 
% ax = gca;
% ax.XLim = [-10 10];
% ax.YLim = [0 10];
% ax.XAxisLocation = 'origin';
% ax.YAxisLocation = "left";
% 
% title('Real L-Shape Trajectory');
% xlabel('X [meters]');
% ylabel('Y [meters]');
% 
% grid on;

% Code for calculating the total distance of the estimated trajectory
output_x = 0;
output_y = 0;
output = 0;
for i=2:length(brushedData(:,1))

    x = abs((brushedData(i,1) - brushedData((i-1),1)));
    y = abs((brushedData(i,2) - brushedData((i-1),2)));

    absoloute = sqrt(x^2 + y^2);
    
    output = output + absoloute;
%     output_x = output_x + abs((brushedData(i,1) - brushedData((i-1),1)));
%     output_y = output_y + abs((brushedData(i,2) - brushedData((i-1),2)));
end
