# from util.enuns import Status, ROSComunicationType, LogColor

variable_template = """self.{_key} = action """

# TODO: Add ros actions or services
class Actions():
    """docstring for Actions."""
    def __init__(self, report):
        self.report = report

    def add_action(self, key, action):
        exec (variable_template.format(_key=key))
        # self.report.add_log(str(ROSComunicationType.ACTION.name) + ' - ' +
        #     key, Status.FOUND.name, color=LogColor.GREEN)
