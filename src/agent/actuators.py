import rospy
import rostopic
import rosservice
import actionlib

from agent.util.enuns import DeviceType, Status, ROSComunicationType, ActionlibState, LogColor
from agent.util.agent_exception import AgentException

topic_publish_template = """
from {_module}.msg import {_class}
self.pub_{_key} = rospy.Publisher('{_node}', {_class}, queue_size=80)
"""

topic_function_template = """
def {_key}(self, msg):
    self.pub_{_key}.publish(msg)
    self.rate.sleep()
"""

service_function_template = """
def {_key}(self, *req):
    from {_msg_module}.srv import {_class}
    rospy.wait_for_service('{_node}')
    sp = rospy.ServiceProxy('{_node}', {_class})
    result = sp(*req)
    return result
"""

action_function_template = """
def {_key}(self, goal):
    from {_module}.msg import {_class}Goal
    from {_module}.msg import {_class}

    client = actionlib.SimpleActionClient('{_node}', {_class})
    client.wait_for_server()
    client.send_goal(goal)
    client.wait_for_result()
    return ActionlibState(client.get_state()).name
"""

setattr_template = """setattr(Actuators, '{_key}', {_key})"""


class Actuators():
    """docstring for Actuators"""

    def __init__(self, report):
        self.rate = rospy.Rate(10)
        self.report = report
        self.actuators_param = rospy.get_param('robot_actuators')
        # print(self.actuators_param)

        self.actuators = self.actuators_param
        # self.actuators = dict.fromkeys(self.actuators_param.keys(), None)
        self.status = dict.fromkeys(self.actuators_param.keys(), Status.NOT_FOUND)

        for key in self.actuators.keys():
            if(self.update(key)):
                self.report.add_log(str(DeviceType.ACTUATOR.name) + ' - ' +
                    key, self.status[key].name ,color=LogColor.GREEN)
            else:
                self.report.add_log(str(DeviceType.ACTUATOR.name) + ' - ' +
                    key, self.status[key].name ,color=LogColor.YELLOW)

    def update(self, key):

        available_topics = rostopic.get_topic_list()
        available_services = rosservice.get_service_list()

        lista0 = [i[0] for i in available_topics[0]]
        lista1 = [i[0] for i in available_topics[1]]
        lista2 = available_services

        service_list = rosservice.get_service_list()


        topic = str('/'+str(self.actuators[key]['node']))
        # print(topic) 

        if(topic in lista0 or topic in lista1 or topic in lista2) :

            actuator = self.actuators_param[key]
            actuator_node = actuator['node']
            actuator_type = actuator['type']
            msg_module, msg_class = actuator_type.split("/")

            if (actuator['comm'] == ROSComunicationType.TOPIC.name):
                exec (topic_function_template.format(_key=key))
                exec (topic_publish_template.format(
                    _key=key, _node=actuator_node,
                    _module=msg_module, _class=msg_class))
                exec (setattr_template.format(_key=key))

            elif (actuator['comm'] == ROSComunicationType.SERVICE.name):
                exec (service_function_template.format(
                    _key=key, _node=actuator_node,
                    _msg_module=msg_module, _class=msg_class))
                exec (setattr_template.format(_key=key))

            elif (actuator['comm'] == ROSComunicationType.ACTION.name):
                msg_class_without_goal = msg_class.replace("Goal", "")
                node_without_goal = actuator_node.replace("/goal", "")

                exec (action_function_template.format(
                    _key=key, _node=node_without_goal,
                    _module=msg_module, _class=msg_class_without_goal))
                exec (setattr_template.format(_key=key))

            self.status[key] = Status.FOUND
            return True
        else:
            self.status[key] = Status.NOT_FOUND
            return False




        # try:
        #     actuator = self.actuators_param[key]
        #     node = actuator['node']
        #     msg_type = actuator['type']
        #     msg_module, msg_class = msg_type.split("/")

        #     if (actuator['comm'] == ROSComunicationType.TOPIC.name):
        #         exec (topic_function_template.format(_key=key))
        #         exec (topic_publish_template.format(
        #             _key=key, _node=node,
        #             _module=msg_module, _class=msg_class))
        #         exec (setattr_template.format(_key=key))

        #     elif (actuator['comm'] == ROSComunicationType.SERVICE.name):
        #         exec (service_function_template.format(
        #             _key=key, _node=node,
        #             _msg_module=msg_module, _class=msg_class))
        #         exec (setattr_template.format(_key=key))

        #     elif (actuator['comm'] == ROSComunicationType.ACTION.name):
        #         msg_class_without_goal = msg_class.replace("Goal", "")
        #         node_without_goal = node.replace("/goal", "")

        #         exec (action_function_template.format(
        #             _key=key, _node=node_without_goal,
        #             _module=msg_module, _class=msg_class_without_goal))
        #         exec (setattr_template.format(_key=key))

        #     self.status[key] = Status.FOUND
        #     return True

        # except Exception as e:
        #     print (e)
        #     self.status[key] = Status.NOT_FOUND
        #     return False
