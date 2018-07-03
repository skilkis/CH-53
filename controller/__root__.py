""" Adds compatibility to be able to import parent-level modules from children when running children as scripts """

import sys
sys.path.insert(0, '..')
