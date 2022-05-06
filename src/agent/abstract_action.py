import abc
from abc import ABCMeta, abstractmethod

class AbstractAction():
    __metaclass__ = ABCMeta

    def __init__(self, robot):
        self.robot = robot

    @abc.abstractmethod
    def execute(self):
        pass
