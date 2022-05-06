import os
import time
import rospy
from termcolor import colored
from agent.util.enuns import LogLevel, LogColor

class Report():
    """docstring for Report."""
    def __init__(self):
        self.robot_ns = rospy.get_namespace().replace('/','')
        if(self.robot_ns == ""):
            self.robot_ns = rospy.get_name().replace('/','')

        self.now = time.strftime("%d:%m:%y-%H:%M:%S", time.localtime(time.time()))
        self.enable = rospy.get_param('report_enable',False)
        self.title = rospy.get_param('report_title','').replace('_','\_')
        self.author = rospy.get_param('report_author','').replace('_','\_')
        self.log = []
        self.pictures = []

    def add_log(self, module, msg, log_level=LogLevel.INFO, color=LogColor.DEFAULT, fig_file=None):
        now = time.strftime("%H:%M:%S", time.localtime(time.time()))
        log_ros = module + ': ' + colored(msg,color.value[0])
        log_tex = '\\lbrack' + str(now) + '\\rbrack ~ ' + module + ': \\textcolor{' + color.value[1] + '}{' + msg + '}'
        if(fig_file):
            log_tex += ' (See Figure \\ref{fig:' + fig_file + '})'
            self.pictures.append([fig_file, module + ': ' + msg])


        if(log_level == LogLevel.DEBUG):
            rospy.logdebug(log_ros)
            self.log.append(LogLevel.DEBUG.name + ' ' + log_tex)
        elif(log_level == LogLevel.INFO):
            rospy.loginfo(log_ros)
            self.log.append(LogLevel.INFO.name + ' ' + log_tex)
        elif(log_level == LogLevel.WARN):
            rospy.logwarn(log_ros)
            self.log.append(LogLevel.WARN.name + ' ' + log_tex)
        elif(log_level == LogLevel.ERROR):
            rospy.logerr(log_ros)
            self.log.append(LogLevel.ERROR.name + ' ' + log_tex)
        elif(log_level == LogLevel.FATAL):
            rospy.logfatal(log_ros)
            self.log.append(LogLevel.FATAL.name + ' ' + log_tex)
        else:
            rospy.loginfo(log_ros)
            self.log.append(LogLevel.INFO.name + ' ' + log_tex)

    def generateTEX(self):
        if(self.enable):

            if (not os.path.exists(self.robot_ns)):
                os.makedirs(self.robot_ns)

            self.file = open(self.robot_ns+'/'+self.robot_ns+'_'+str(self.now)+'.tex', 'w')
            self.file.write('\\documentclass{article}\n')
            self.file.write('\\usepackage[utf8]{inputenc}\n')
            self.file.write('\\usepackage{graphicx}\n')
            self.file.write('\\usepackage[dvipsnames]{xcolor}\n')
            self.file.write('\\usepackage{url}\n')

            self.file.write('\\begin{document}\n')
            self.file.write('\\title{'+self.title+'}\n')
            self.file.write('\\author{'+self.author+'}\n')
            self.file.write('\\maketitle\n')
            self.file.write('\\section*{Log:}\n')
            self.file.write('\\begin{itemize}\n')
            for i in self.log:
                self.file.write('\\item '+i+'\n')
            self.file.write('\\end{itemize}\n')
            self.file.write('\\section*{Pictures:}\n')
            for i in self.pictures:
                self.file.write('\\begin{figure}[ht]\n')
                self.file.write('\\includegraphics[height=0.4\\textheight]{'+i[0]+'}\n')
                self.file.write('\\caption{'+i[1]+'}\n')
                self.file.write('\\label{fig:'+i[0]+'}\n')
                self.file.write('\\end{figure}\n')
            self.file.write('\\end{document}\n')
            self.file.close()

            self.generatePDF()


    def generatePDF(self):
        if(self.enable):
            # TODO:remove system call
            os.system('pdflatex '+self.robot_ns+'/'+self.robot_ns+'_'+str(self.now)+'.tex > /dev/null 2>&1')
