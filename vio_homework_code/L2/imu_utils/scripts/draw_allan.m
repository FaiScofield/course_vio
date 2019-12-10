clear 
close all

dt = dlmread('../data/data_vio_imu_1_gyr_t.txt');         
data_x = dlmread('../data/data_vio_imu_1_gyr_x.txt'); 
data_y= dlmread('../data/data_vio_imu_1_gyr_y.txt'); 
data_z = dlmread('../data/data_vio_imu_1_gyr_z.txt'); 
data_draw=[data_x data_y data_z]*3.1415926/180/3600;

data_sim_x= dlmread('../data/data_vio_imu_1_sim_gyr_x.txt'); 
data_sim_y= dlmread('../data/data_vio_imu_1_sim_gyr_y.txt'); 
data_sim_z= dlmread('../data/data_vio_imu_1_sim_gyr_z.txt'); 
data_sim_draw=[data_sim_x data_sim_y data_sim_z]*3.1415926/180/3600 ;


figure
loglog(dt, data_draw , 'o');
% loglog(dt, data_sim_draw , '-');
xlabel('time:sec');                
ylabel('Sigma:rad/s');             
% legend('x','y','z');      
grid on;                           
hold on;                           
loglog(dt, data_sim_draw , '-');
