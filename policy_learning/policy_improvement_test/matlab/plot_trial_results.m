function plot_trial_results()

clear all;
close all;

disp('-----------------------------------------------------------------------------------');

% dir = '/u/pastor/TestData/dmp_learn';
dir = '/home/pastor/workspace/policy_improvement_test/library/dmp_test_learn';

log_info = load(sprintf('%s/log_info.dat', dir));

num_steps = log_info(1);
num_trials = log_info(2);
num_rollouts = log_info(3);
num_rfs = log_info(4);
num_reused_trials = log_info(5);
num_dimensions = log_info(6);

log_data = load(sprintf('%s/log_data.dat', dir));
log_reward = load(sprintf('%s/log_reward.dat', dir));
log_costs = load(sprintf('%s/log_costs.dat', dir));
log_goals = load(sprintf('%s/log_goals.dat', dir));
log_theta = load(sprintf('%s/log_theta.dat', dir));

size(log_data)

waypoint_timings = log_goals(:,1);
waypoint_index = waypoint_timings .* num_steps;
waypoints = log_goals(:,2);
goals = log_goals(:,3);

% size(log_data)

time = log_data(:,1);
positions = log_data(:,2:1+num_dimensions);
velocities = log_data(:,2+num_dimensions:1+2*num_dimensions);
canonical_system_s = log_data(:,end);

if num_steps~=(length(time)/num_trials)
    error('Number of steps of the trajectories (%i) does not correspond with the number of steps read from the info file (%i).', num_steps, (length(time)/num_trials));
end

thick_linewidth = 1.8;
thin_linewidth = 0.8;

figure(1)
hold on;
box on;

num_plots = 3;
for dim=1:num_dimensions
    subplot(num_plots, num_dimensions, dim)
    hold on;
    box on;
    for i=1:num_trials
        trial_start_index = 1+(i-1)*num_steps;
        trial_end_index = i*num_steps;
        % for j=1:num_rollouts
        if i==num_trials
            plot(positions(trial_start_index:trial_end_index, dim), 'r', 'LineWidth', thick_linewidth);                        
        elseif i==1
            plot(positions(trial_start_index:trial_end_index, dim), 'm', 'LineWidth', thick_linewidth);                        
        else
            plot(positions(trial_start_index:trial_end_index, dim), 'LineWidth', thin_linewidth);            
        end
        plot(waypoint_index, waypoints, 'go', 'MarkerSize', 6, 'LineWidth', 3);
        plot(num_steps-1,goals(dim), 'go', 'MarkerSize', 6, 'LineWidth', 3);
        title(sprintf('Positions of DMP %i', dim),'Linewidth',16);
        % end
    end
    hold off;

    subplot(num_plots, num_dimensions, num_dimensions+dim)
    hold on;
    box on;
    for i=1:num_trials
        trial_start_index = 1+(i-1)*num_steps;
        trial_end_index = i*num_steps;
        % for j=1:num_rollouts
        if i==num_trials
            plot(velocities(trial_start_index:trial_end_index, dim), 'r', 'LineWidth', thick_linewidth);                        
        elseif i==1
            plot(velocities(trial_start_index:trial_end_index, dim), 'm', 'LineWidth', thick_linewidth);                        
        else
            plot(velocities(trial_start_index:trial_end_index, dim), 'LineWidth', thin_linewidth);
        end
        title(sprintf('Velocities of DMP %i', dim),'Linewidth',16);
        % end
    end
    hold off;

    subplot(num_plots, num_dimensions, 2*num_dimensions+dim)
    hold on;
    box on;
    for i=1:num_trials
        trial_start_index = 1+(i-1)*num_rfs;
        trial_end_index = i*num_rfs;
        % for j=1:num_rollouts

        if i==num_trials
            plot(log_theta(trial_start_index:trial_end_index, dim), '.r', 'LineWidth', thick_linewidth);                        
        elseif i==1
            plot(log_theta(trial_start_index:trial_end_index, dim), '.m', 'LineWidth', thick_linewidth);                        
        else
            plot(log_theta(trial_start_index:trial_end_index, dim), '.', 'LineWidth', thin_linewidth);
        end
        title(sprintf('Thetas of DMP %i', dim),'Linewidth',16);
        % end
    end
    xlim([1 num_rfs]);
    hold off;
    
end
hold off;

% fullscreen;

figure(2)
hold on;
num_plots = 2;

subplot(num_plots, 1, 1)
hold on;
plot(log_reward);
box on;
hold off;

subplot(num_plots, 1, 2)
hold on;
for i=1:num_trials
    trial_start_index = 1+(i-1)*num_steps;
    trial_end_index = i*num_steps;
    plot(canonical_system_s(trial_start_index:trial_end_index));
end
box on;
hold off;    

hold off;
% fullscreen;