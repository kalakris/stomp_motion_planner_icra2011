function combine_clmc_files_desired_vs_actual(varargin)

if(length(varargin) < 2)
    % dir = '/u/pastor/workspace/dmp_motion_generation_test/library';
    dir = '/u/pastor/workspace/dmp_motion_controller_test/library';
else
    dir = varargin{2};
end

debug_dir = 'dmp_test_debug';

if(length(varargin) < 1)
    ids = [1, 2];
else
    ids = varargin{1};
end

num_ids = length(ids) * 2;
copy_ids = zeros(num_ids, 1);
for i=1:length(ids)
    copy_ids(i*2 -1) = ids(i);
    copy_ids(i*2) = ids(i);
end

clear ids;
ids = copy_ids;

filenames_postfixes = cell(2);

version = cell(num_ids);
for i=1:num_ids
    if (mod(i, 2) == 1)
        version{i} = 'icra2009';
    else
        version{i} = 'nips2003';
        version{i} = 'icra2009';
    end
end

for i=1:num_ids
    filenames_postfixes{1} = {sprintf('%s/%s/item_%i_actual_0_%s_%i_0.traj', dir, debug_dir, ids(i), version{i}, ids(i)), sprintf('')};
    filenames_postfixes{2} = {sprintf('%s/%s/item_%i_desired_0_%s_%i_0.traj', dir, debug_dir, ids(i), version{i}, ids(i)), sprintf('')};
    combined_filename = sprintf('%s/%s/debug_desired_vs_actual_%s_%i.traj', dir, debug_dir, version{i}, ids(i));
    combine_clmc_files(filenames_postfixes,combined_filename);
    num_ids = num_ids+1;
end
