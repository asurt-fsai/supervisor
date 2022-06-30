from dataclasses import dataclass
from typing import List

import roslaunch
import time
import rospy
from asurt_msgs.msg import Status

class Module:
    """
        Holds the required launch file args of a module for inspection.\n
        You can pass a launch file for an accessory (i.e rviz, plotter, etc.)

        Attributes
        ----------
        pkg : str
            The name of the package.
        launch : str
            The name of the launch file.
        topics : List[str], optional
            The name of the topics that node publishes.
    """
    
    def __init__(self, pkg, launch_file, heartbeat=None):
        self.pkg = pkg
        self.launch_file = launch_file
        if heartbeat is not None:
            rospy.Subscriber(heartbeat, Status, self.heartbeat_cb)

        self.module_handle = None
        self.state = 4  # Shutdown
        self.count_since_last = 0
        self.rate = 0
        self.schedule_restart = 0
    
    def restart(self):
        self.shutdown()
        self.launch()
    
    def shutdown(self):
        if self.module_handle is not None:
            self.module_handle.shutdown()
            self.module_handle = None
        self.state = 4

    def launch(self):
        if self.pkg is None or self.launch_file is None:
            return None, None
        self.state = 0
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        def launch_file(pkg: str, file: str):
            if (None in [pkg, file]):
                return None

            cli_args = [pkg, file]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)

            parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            parent.start()

            return parent
            
        self.module_handle = launch_file(self.pkg, self.launch_file)
    
    def heartbeat_cb(self, msg):
        self.state = msg.status
        self.count_since_last += 1
    
    def update(self):
        if self.schedule_restart == 1:
            self.restart()
            self.schedule_restart = 0
        if self.count_since_last==0 and self.state == 2:
            self.state = 5  # unresponsive
        if self.rate == 0:
            self.rate = self.count_since_last
        else:
            beta = 0.3
            self.rate = beta*self.rate + (1-beta)*self.count_since_last
        self.count_since_last = 0


class Inspector(object):
    """
        Inspects a given list of modules (pipeline).

        Attributes
        ----------
        modules : List[Module]
            A pipline of modules.
    """


    def __init__(self, modules: List[Module]):
        self.modules: List[Module] = modules

    def manual_inspect(self):
        """
            Manually, inpect the module pipline. User input directs 
            the actions taken for each module.
        """

        for module in self.modules:
            # until passed
            while True:
                # launch the module, and get handles
                module.launch()

                # check the module state
                if module is None:
                    print("NO MODULE...")

                
                # take next action from the manual inspector
                prompt = "Next action?\nk -> confirmed but keep inspection view \nr -> restart module\n>\n "
                action = input(prompt)

                if action.lower() == 'k':
                    break
                
                # restart the module
                if action.lower() == 'r':
                    module.shutdown()

    def auto_launch(self):
        """
            Automatically, launch the module pipline.
        """
        for module in self.modules:
            module.launch()

    def update(self):
        for module in self.modules:
            module.update()

    def get_data(self):
        data = []
        states = ['starting', 'ready','running','error','shutdown','unresponsive']
        r = 1
        for module in self.modules:
            data.append([module.pkg, states[module.state],'{:.2f} hz'.format(module.rate), module.state])
            r += 1
        return data

