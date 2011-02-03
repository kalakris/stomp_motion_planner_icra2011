function lwr_plot_test_results()

root = '/home/pastor/workspace';

package_name = 'lwr';
library_directory_name = 'library';
data_directory_name = 'data';

dir = sprintf('%s/%s/%s', root, package_name, data_directory_name);

test_x = load(sprintf('%s/test_x.txt', dir));
test_y = load(sprintf('%s/test_y.txt', dir));
test_xq = load(sprintf('%s/test_xq.txt', dir));
test_yp = load(sprintf('%s/test_yp.txt', dir));
test_yp_copy = load(sprintf('%s/test_yp_copy.txt', dir));
basis_function_matrix = load(sprintf('%s/basis_function_matrix.txt', dir));

figure(1)
hold on;
subplot(3, 1, 1)
hold on;
plot(test_x, test_y, 'bo');
plot(test_xq, test_yp, 'g.');
plot(test_xq, test_yp_copy, 'r.');
title(sprintf('blue: target\nred: prediction from LWR model read from file'));
box on;
hold off;

subplot(3, 1, 2)
hold on;
plot(basis_function_matrix);
box on;
hold off;

subplot(3, 1, 3)
hold on;
plot(test_x);
box on;
hold off;

hold off;
box on;