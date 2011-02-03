function plot_dmp(varargin)

root = '/home/pastor/workspace';

package_name = 'dmp_motion_generation_test';
library_base_name = 'library';

%data_dir = 'dmp_test_lib';
%learn_dir = 'dmp_test_learn';
%debug_dir = 'dmp_test_debug';
lib_dir = 'dmp_test_lib';

global item_dir_name;
global trans_prefix;
global trans_postfix;

item_dir_name = 'item_';
trans_prefix = 'trans_';
trans_postfix = '.dcpt';


lib_dir_is_set = 0;
abs_lib_dir = root;

if (exist(abs_lib_dir, 'file'))
    abs_lib_dir = sprintf('%s/%s', abs_lib_dir, package_name);
    if(exist(abs_lib_dir, 'file'))
        abs_lib_dir = sprintf('%s/%s', abs_lib_dir, library_base_name);
        if(exist(abs_lib_dir, 'file'))
            abs_lib_dir = sprintf('%s/%s',abs_lib_dir,lib_dir);
            if(exist(abs_lib_dir, 'file'))
                fprintf('Library directory is set to %s.\n',abs_lib_dir);
                lib_dir_is_set = 1;
            else
                fprintf('Directory %s does not exist\n', abs_lib_dir);
            end
        else
            fprintf('Directory %s does not exist\n', abs_lib_dir);
        end
    else
        fprintf('Directory %s does not exist\n', abs_lib_dir);
    end
else
    fprintf('Directory %s does not exist\n', abs_lib_dir);
end

if(~lib_dir_is_set)
    error('Library directory is not set properly.');
end

if(isempty(varargin))
    error('No ids specified.');
end

num_ids = length(varargin);
item_dirs = cell(num_ids);
num_trans = cell(num_ids);
for i=1:num_ids
    item_dirs{i} = sprintf('%s/%s%i', abs_lib_dir, item_dir_name, varargin{i});
    if(~exist(item_dirs{i}, 'file'))
        error('Directory %s for id %i does not exist.',item_dirs{i}, varargin{i});
    end
    fprintf('item_dir{%i}: %s\n', i, item_dirs{i});

    num_trans{i} = 0;
    found = 1;
    while(found == 1)
        filename = sprintf('%s/%s%i%s', item_dirs{i}, trans_prefix, num_trans{i}, trans_postfix);
        if(exist(filename, 'file'))
            num_trans{i} = num_trans{i} + 1;
            fprintf('found %s .\n', filename);
        else
            % fprintf('%s does not exist.\n', filename);
            found = 0;
        end
    end

    trans = linspace(1, num_trans{i}, num_trans{i});
    plot_dmp_figure(item_dirs{i}, trans, varargin{i});
end

function plot_dmp_figure(dir, trans, id)

    global trans_prefix;

    fprintf('ID = %i\n', id);
    
    num_trans = length(trans);
    
    num_rows = 3;
    
    for j=1:num_trans
        center_filename = sprintf('%s/%s%i_centers.txt', dir, trans_prefix, j-1);
        centers = load(center_filename);
        theta_filename = sprintf('%s/%s%i_thetas.txt', dir, trans_prefix, j-1);
        thetas = load(theta_filename);
        psi_filename = sprintf('%s/%s%i_psi.txt', dir, trans_prefix, j-1);
        psi = load(psi_filename);
        sigma_filename = sprintf('%s/%s%i_sigma.txt', dir, trans_prefix, j-1);
        sigma = load(sigma_filename);
        num_rfs_filename = sprintf('%s/%s%i_num_rfs.txt', dir, trans_prefix, j-1);
        num_rfs = load(num_rfs_filename);

        if(length(thetas) ~= num_rfs)
            error('Number of rfs (%i) does not match number of thetas (%i)', num_rfs, length(thetas));
        end
        if(length(centers) ~= num_rfs)
            error('Number of rfs (%i) does not match number of thetas (%i)', num_rfs, length(centers));
        end
    
        psi_is_valid = 1;
        if(isempty(psi))
           disp('Warning: PSI matrix read from file is empty. Not plotting it...');
           psi_is_valid = 0;          
        end

        if(psi_is_valid)
            psi_matrix = zeros(size(psi));
            x_vector = linspace(0,1,length(psi_matrix(:,1)));
            for k=1:num_rfs
                psi_matrix(:,k) = exp(- (1/sigma) .* (x_vector-centers(k)).^2);
            end
        end
        
        figure(id)
        hold on;
        box on;

        row = 0;
        subplot(num_rows, num_trans, j+(row*num_trans))
        hold on;
        box on;
        if(psi_is_valid)
            plot(x_vector, psi_matrix);
            xlim([min(x_vector) max(x_vector)]);
        end
        title(sprintf('generated psi matrix of trans %i', j), 'FontSize', 14);
        hold off;

        row = row+1;
        subplot(num_rows, num_trans, j+(row*num_trans))
        hold on;
        box on;
        if(psi_is_valid)
            plot(x_vector, psi);
            xlim([min(x_vector) max(x_vector)]);
        end
        title(sprintf('loaded psi matrix of trans %i', j), 'FontSize', 14);
        hold off;

        row = row+1;
        subplot(num_rows, num_trans, j+(row*num_trans))
        hold on;
        box on;
        plot(thetas, 'o', 'MarkerSize', 4);
        xlim([1 length(thetas)]);
        title('thetas', 'FontSize', 14);
        hold off;

        hold off;

    end








