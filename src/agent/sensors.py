import rospy
import rostopic

from agent.util.enuns import DeviceType, Status, LogColor
from agent.util.agent_exception import AgentException

import_template = """
from {_module}.msg import {_class}
"""
callback_template = """
def callback_{_key}(self, data):
    self.sensors['{_key}'] = data
    self.status['{_key}'] = Status.OK
"""
setattr_template = """
setattr(Sensors, 'callback_{_key}', callback_{_key})
"""
subscriber_template = """
rospy.Subscriber('{_node}', {_class}, self.callback_{_key})
"""

class Sensors():
    """docstring for Sensors"""

    def __init__(self, report):
        self.rate = rospy.Rate(10)
        self.report = report
        self.sensors_param = rospy.get_param('robot_sensors')
        # print(self.sensors_param)

        self.sensors = self.sensors_param
        # self.sensors = dict.fromkeys(self.sensors_param.keys(), self.sensors_param.values())
        self.status = dict.fromkeys(self.sensors_param.keys(), Status.NOT_FOUND)

        for key in self.sensors.keys():
            if(self.update(key)):
                self.report.add_log(str(DeviceType.SENSOR.name) + ' - ' +
                    key, self.status[key].name, color=LogColor.GREEN)
            else:
                self.report.add_log(str(DeviceType.SENSOR.name) + ' - ' +
                    key, self.status[key].name, color=LogColor.YELLOW)

    def update(self, key):

        available_topics = rostopic.get_topic_list()

        lista0 = [i[0] for i in available_topics[0]]
        lista1 = [i[0] for i in available_topics[1]]

        topic = str('/'+str(self.sensors[key]['node']))
        # print(topic) 

        if(topic in lista0 or topic in lista1):

            sensor = self.sensors_param[key]
            sensor_node = sensor['node']
            sensor_type = sensor['type']
            msg_module, msg_class = sensor_type.split("/")

            exec (import_template.format(_module=msg_module, _class=msg_class))
            exec (callback_template.format(_key=key))
            exec (setattr_template.format(_key=key))
            exec (subscriber_template.format(_node=sensor_node, _class=msg_class, _key=key))

            self.status[key] = Status.FOUND
            return True
        else:
            self.status[key] = Status.NOT_FOUND
            return False

