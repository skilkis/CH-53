#!/usr/bin/env python
# -*- coding: utf-8 -*-

__author__ = ["aubricus", "San Kilkis"]

import sys
from timeit import default_timer as timer
import threading
from time import sleep

# TODO Comment code fully


class ProgressBar(threading.Thread):
    """ Creates a progress bar when called that displayes in the terminal. Can be updated w/ built-in methods
    :method:`update` or :method:`update_loop`. The latter makes it easy to update within a for-loop as long as
    a counting variable is present.

    WARNING: If you are using IDLE the progress bar will not display correctly due to string incompabilities. Adapted
    from: https://gist.github.com/aubricus/f91fb55dc6ba5557fbab06119420dd6a#file-print_progress-py

    :param header: Explanation of the process
    :type header: str

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

    :param threaded: Transfers writing procedure to a parallel thread if time between iterations is long
    :param threaded: bool
    """

    def __init__(self, header=None, prefix='Progress', suffix='Complete', decimals=1,
                 bar_length=50, show_time=True, threaded=False):

        threading.Thread.__init__(self)  # Initializes in a new thread

        # Inputs
        self.header = header
        self.prefix = prefix
        self.suffix = suffix
        self.decimals = decimals
        self.bar_length = bar_length
        self.show_time = show_time
        self.threaded = threaded

        # Formatting Attribute
        self._str_format = "{0:." + str(self.decimals) + "f}"
        self.update_msg = ''

        # Initial Conditions
        self.start_time = timer()
        self.percent_complete = 0
        self.complete = False
        self.update_time = None
        self.final_printed = False

        sys.stdout.write('\n%s\n' % self.header if self.header is not None else '\n')
        self._writer()  # Displays initial conditions once
        if self.threaded:
            self.start()  # Automatically starts the multi-threaded timer based on input

    def run(self):
        """ Executes a running-process in a new thread that updates the progress-bar with the elapsed time. Note this
        process is only executed if :param:`threaded` is True. """
        refresh_rate = 3  # [Hz]
        while not self.complete:
            sleep(1/float(refresh_rate))
            self._writer()

    @property
    def current_time(self):
        return timer()

    @property
    def stop_time(self):
        return self.update_time if self.complete else None

    @property
    def elapsed_time(self):
        """

        :return: Formatted string depicting the time elapsed after :class:`ProgressTimer` has been called
        :rtype: str
        """
        elapsed_time = self.current_time - self.start_time if not self.complete else self.stop_time - self.start_time
        return 'Elapsed Time: %3.4f [s],' % elapsed_time if self.show_time else ''

    def _writer(self):
        """ Responsible for progressing and writing the progress bar to the terminal

        :return:
        """
        filled_length = int(round(self.bar_length * (self.percent_complete / 100.)))
        bar = '#' * filled_length + '-' * (self.bar_length - filled_length)
        percent_str = self._str_format.format(self.percent_complete)

        sys.stdout.write('\r%s %s |%s| %s%s %s %s' % (self.elapsed_time, self.prefix, bar, percent_str, '%',
                                                      self.suffix, self.update_msg))
        if self.complete:
            sys.stdout.write('\n\n')

    def update_loop(self, iteration, total, update_msg=''):
        """ Updates the progress bar w/ the current loop iteration.

        :param iteration: Current iteration index
        :type iteration: int

        :param total: Total iterations index, if range(0, N) is used in the loop the passed value must be N-1
        :type total: int

        :param update_msg: Optional message to be displayed during update of progress
        :type update_msg: str
        """

        setattr(self, 'update_time', self.current_time)  # Overwriting last update time

        iter2percent = (100. * (iteration / float(total)))
        setattr(self, 'percent_complete', iter2percent)
        setattr(self, 'complete', True if self.percent_complete == 100 else False)
        if update_msg is not '':
            setattr(self, 'update_msg', '(%s)' % update_msg)
        self._force_write()

    def update(self, percent_complete, update_msg=''):
        setattr(self, 'update_time', self.current_time)

        setattr(self, 'percent_complete', percent_complete)
        setattr(self, 'complete', True if self.percent_complete == 100 else False)
        if update_msg is not '':
            setattr(self, 'update_msg', '(%s)' % update_msg)
        self._force_write()

    def _force_write(self):
        if not self.threaded:
            self._writer()


if __name__ == '__main__':
    start = timer()
    prog = ProgressBar('Sample Process')
    for i in range(0, 6):
        sleep(1)
        prog.update_loop(i, 5, update_msg='%1.2f' % i)




