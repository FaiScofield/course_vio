clear 
close all

dt = dlmread('../data/data_vio_imu_1_gyr_t.txt');         
data_x = dlmread('../data/data_vio_imu_1_gyr_x.txt'); 
data_y= dlmread('../data/data_vio_imu_1_gyr_y.txt'); 
data_z = dlmread('../data/data_vio_imu_1_gyr_z.txt'); 
data_draw=(data_x+data_y+data_z)/3 /3600 /180 *3.1415926;

data_sim_x= dlmread('../data/data_vio_imu_1_sim_gyr_x.txt'); 
data_sim_y= dlmread('../data/data_vio_imu_1_sim_gyr_y.txt'); 
data_sim_z= dlmread('../data/data_vio_imu_1_sim_gyr_z.txt'); 
data_sim_draw=(data_sim_x+data_sim_y+data_sim_z)/3 /3600 /180 *3.1415926;

figure
loglog(dt, data_draw , 'r+');
xlabel('time: sec');                
ylabel('Sigma: rad/s');   
title('vio\_imu\_1')          
% legend('raw\_data\_gyr','sim\_data\_gyr', 'raw\_data\_acc','sim\_data\_acc');    
grid on;  
hold on;  
loglog(dt, data_sim_draw , 'r-');
           
dt = dlmread('../data/data_vio_imu_1_acc_t.txt');         
data_x = dlmread('../data/data_vio_imu_1_acc_x.txt'); 
data_y= dlmread('../data/data_vio_imu_1_acc_y.txt'); 
data_z = dlmread('../data/data_vio_imu_1_acc_z.txt'); 
data_draw=(data_x+data_y+data_z)/3 ;

data_sim_x= dlmread('../data/data_vio_imu_1_sim_acc_x.txt'); 
data_sim_y= dlmread('../data/data_vio_imu_1_sim_acc_y.txt'); 
data_sim_z= dlmread('../data/data_vio_imu_1_sim_acc_z.txt'); 
data_sim_draw=(data_sim_x+data_sim_y+data_sim_z)/3 ;

% figure
loglog(dt, data_draw , 'b+');                           
loglog(dt, data_sim_draw , 'b-');
legend('raw\_data\_gyr','sim\_data\_gyr', 'raw\_data\_acc','sim\_data\_acc');  
