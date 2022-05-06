

import abc
from abc import ABCMeta, abstractmethod

from agent.util.enuns import DeviceType, Status, LogLevel, LogColor
from agent.util.agent_exception import AgentException
from agent.world_model.world_model import WorldModel

from agent.report import Report
from agent.sensors import Sensors
from agent.actuators import Actuators
from agent.actions import Actions

import _thread
import rospy

class AbstractAgent(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        # set private attributs
        self.__world_model__ = WorldModel()
        self.__update_time__ = rospy.get_param('update_time', 0.9)
        self.__max_trys__ = rospy.get_param('max_trys', 10)
        self.__report__ = Report()
        self.__sensors__ = Sensors(self.__report__)
        self.__actuators__ = Actuators(self.__report__)

        # main agent attributs
        self.actions = Actions(self.__report__)

        # start thread for world model update
        _thread.start_new_thread(self.__update_world_model__, ())

    def generatePDF(self):
        self.__report__.generateTEX()
        self.__report__.generatePDF()

    def add_log(self, module, msg, log_level=LogLevel.INFO, color=LogColor.DEFAULT, fig_file=None):
        self.__report__.add_log(module, msg, log_level, color, fig_file)

    def get_sensors(self, sensor):
        #
        if (self.__sensors__.status[sensor] == Status.NOT_FOUND):
            for i in range(1,self.__max_trys__+1):
                rospy.sleep(1)
                if(self.__sensors__.update(sensor)):
                    self.add_log(str(DeviceType.SENSOR.name) + ' - ' +
                        sensor, Status.FOUND.name,color=LogColor.GREEN)
                    break
                else:
                    self.add_log(str(DeviceType.SENSOR.name) + ' - ' +
                        sensor,'Trying to find ('+ str(i) +'/' +
                            str(self.__max_trys__) + ')',color=LogColor.YELLOW)

        #
        if (self.__sensors__.status[sensor] == Status.FOUND):
            for i in range(1,self.__max_trys__+1):
                rospy.sleep(1)
                if(self.__sensors__.status[sensor] == Status.OK):
                    break
                else:
                    self.add_log(str(DeviceType.SENSOR.name) + ' - ' +
                        sensor,'Waiting firt msg ('+ str(i) +'/' +
                            str(self.__max_trys__) + ')',color=LogColor.YELLOW)

        return self.__sensors__.sensors[sensor]

    def get_actuators(self):
        return self.__actuators__

    def __update_world_model__(self):
        while True:
            if (not rospy.is_shutdown()):
                rospy.sleep(self.__update_time__)
                self.cicle()
            else:
                break

    @abc.abstractmethod
    def cicle(self):
        pass
