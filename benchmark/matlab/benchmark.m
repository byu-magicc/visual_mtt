% If true, the utilization will be converted to seconds; otherwise
% it will be in percentage of time available
seconds = false;

% Open the file
filename = "benchmark1.csv";
path = pwd;
path = path(1:end-6);
path = path+"results/";
filename = path+filename;
data = csvread(filename);



%%  Extract data 

% Get arrays of data
siz = size(data,1);
range = siz-2;
num_tracks = data(1:range,1);
feature_manager_util=data(1:range,2);
homography_manager_util = data(1:range,3);
measurement_generation_util = data(1:range,4);
other_util = data(1:range,5);
rransac_util = data(1:range,6);
total_util = data(1:range,7);
time_available_util = data(1:range,8);
num_measurements = data(1:range,9);

% Get averages
avg_num_tracks = data(siz-1,1);
avg_feature_manager_util = data(siz-1,2);
avg_homography_manager_util = data(siz-1,3);
avg_measurement_generation_util = data(siz-1,4);
avg_other_util = data(siz-1,5)
avg_rransac_util = data(siz-1,6);
avg_total_util = data(siz-1,7);
avg_time_available = data(siz-1,8);
avg_num_rransac_meas = data(siz-1,9);

% Get cuda
cuda = logical(data(siz,1));

if (seconds)
 
    feature_manager_util = feature_manager_util.*time_available_util./100;
    homography_manager_util = homography_manager_util.*time_available_util./100;
    measurement_generation_util = measurement_generation_util.*time_available_util./100;
    other_util = other_util.*time_available_util./100;
    rransac_util = rransac_util.*time_available_util./100;
    total_util = total_util.*time_available_util./100;
    
    avg_feature_manager_util = avg_feature_manager_util.*avg_time_available./100;
    avg_homography_manager_util = avg_homography_manager_util.*avg_time_available./100;
    avg_measurement_generation_util = avg_measurement_generation_util.*avg_time_available./100;
    avg_other_util = avg_other_util.*avg_time_available./100;
    avg_rransac_util = avg_rransac_util.*avg_time_available./100;
    avg_total_util = avg_total_util.*avg_time_available./100;
    
else
    time_available_util = ones(length(time_available_util))*100;
    avg_time_available = 100;
end

%% Plot data

% Plot utilization
figure(1),clf;
subplot(2,1,1);
hold on;
plot(feature_manager_util);
plot(homography_manager_util);
plot(measurement_generation_util);
plot(other_util)
plot(rransac_util);
plot(total_util);
plot(time_available_util);

title('Utilization')
legend('Feature Manager', 'Transform Manager', 'Measurement Manager', 'Other', 'RRANSAC', 'Total Utilization', 'Time Available');
xlabel('Message');
if (seconds)
    ylabel('Seconds');
else
    ylabel('Percentage');
end

% Plot other information
subplot(2,1,2);
hold on
plot(num_tracks)
plot(num_measurements)
legend('Num Tracks', 'Num Measurements');
xlabel('Messages')
title('Other Information')

% Plot average utilization
figure(2),clf;
subplot(1,2,1)
c = categorical({'Feature Manager', 'Homography Manager', 'Source Manager', 'Other', 'RRANSAC', 'Total Utilization', 'Time Available'});
y = [avg_feature_manager_util, avg_homography_manager_util, avg_measurement_generation_util,avg_other_util, avg_rransac_util,avg_total_util,avg_time_available];
bar(c,y);
title('Avg Utilizatioin');
if (seconds)
    ylabel('Seconds');
else
    ylabel('Percentage');
end


% Plot average other information
subplot(1,2,2)
c = categorical({'Num Tracks', 'Num Measurements'});
y = [ avg_num_tracks, avg_num_rransac_meas];
bar(c,y);
title('Avg Other Information')






