import csv
from matplotlib import pyplot as plt
import numpy as np

# 0)Iteration 1)throttle 2)brake 3)steer 4)linear_v_error 5)angular_v_error 6)cte 7)delta_t 8)processing_time 9)avg_proc_time

def main():
    do = (3, 4, 5, 6)
    max_entries = 1800
    count = 0
    with open('../../../../../.ros/charting_data.txt', 'r') as csvfile:
        file_reader = csv.reader(csvfile, delimiter=' ')
        is_header = True
        for line in file_reader:
            if is_header:
                header = line
                y = []
                for i in xrange(len(header)):
                    y.append([])
                is_header = False
                continue
            for i in xrange(len(header)):
                y[i].append(float(line[i]))
            count += 1
            if count == max_entries:
                break

    plt.ion()
    n_categories = len(do)
    n_entries = len(y[0])
    fig, axes = plt.subplots(nrows=n_categories)
    x = list(xrange(n_entries))
    for i in xrange(n_categories):
        category_i = do[i]
        axes[i].plot(x,y[category_i])
        axes[i].text(0, 0, header[category_i], fontdict={'weight': 'bold'})
        # y_bottom, y_top = axes[i].get_ylim()
        # axes[i].yaxis.set_ticks(np.arange(y_bottom, y_top, (y_top-y_bottom)/4))
        axes[i].grid()
    plt.show()
    while True:
        plt.pause(1)


if __name__ == '__main__':
    main()
