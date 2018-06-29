#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = ["aubricus"]

import sys
from timeit import default_timer as timer


def print_progress(iteration, total, start_time=timer(), prefix='Progress', suffix='Complete', decimals=1,
                   bar_length=50, show_time=True):
    """ Creates a progress bar when called in a loop in a loop to create terminal progress bar, adapted from:
    https://gist.github.com/aubricus/f91fb55dc6ba5557fbab06119420dd6a#file-print_progress-py

    :param iteration: Current iteration index
    :type iteration: int

    :param total: Total iterations index, if range(0, N) is used in the loop the passed value must be N-1
    type total: int

    :param start_time: Prefix string in front of progress bar
    :type start_time: float

    :param prefix: Prefix string in front of progress bar
    :type prefix: str

    :param suffix: Suffix string behind progress bar
    :type suffix: str

    :param decimals: Positive number of decimals in percent complete
    :type decimals: int

    :param bar_length: Character length of bar
    :type bar_length: int

    :param show_time: Display elapsed time in seconds
    :param show_time: bool
    """

    current_time = timer()
    elapsed_time = 'Elapsed Time: %3.4f [s],' % (current_time - start_time) if show_time else ''

    str_format = "{0:." + str(decimals) + "f}"
    percents = str_format.format(100 * (iteration / float(total)))
    filled_length = int(round(bar_length * iteration / float(total)))
    bar = '#' * filled_length + '-' * (bar_length - filled_length)

    sys.stdout.write('\r%s %s |%s| %s%s %s' % (elapsed_time, prefix, bar, percents, '%', suffix))

    if iteration == total:
        sys.stdout.write('\n')


if __name__ == '__main__':
    from time import sleep
    for i in range(0, 20):
        print_progress(i, 19)
        sleep(1)



