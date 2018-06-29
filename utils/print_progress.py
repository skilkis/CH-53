#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = ["aubricus"]

import sys


def print_progress(iteration, total, prefix='Progress', suffix='Complete', decimals=1, bar_length=100):
    """ Creates a progress bar when called in a loop in a loop to create terminal progress bar, adapted from:
    https://gist.github.com/aubricus/f91fb55dc6ba5557fbab06119420dd6a#file-print_progress-py

    :param iteration: Current iteration index
    :type iteration: int

    :param total: Total iterations index
    type total: int

    :param prefix: Prefix string in front of progress bar
    :type prefix: str

    :param suffix: Suffix string behind progress bar
    :type suffix: str

    :param decimals: Positive number of decimals in percent complete
    :type decimals: int

    :param bar_length: Character length of bar
    :type bar_length: int
    """

    str_format = "{0:." + str(decimals) + "f}"
    percents = str_format.format(100 * (iteration / float(total)))
    filled_length = int(round(bar_length * iteration / float(total)))
    bar = '█' * filled_length + '-' * (bar_length - filled_length)

    sys.stdout.write('\r%s |%s| %s%s %s' % (prefix, bar, percents, '%', suffix)),

    if iteration == total:
        sys.stdout.write('\n')
        sys.stdout.flush()

