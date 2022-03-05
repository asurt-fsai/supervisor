from dataclasses import dataclass
from typing import List
import roslaunch


@dataclass
class Module(object):
    """
        Holdes the required launch file args of a module for inspection.\n
        You can pass a launch file for an accessory (i.e rviz, plotter, etc.)

        Attributes
        ----------
        pkg : str
            The name of the package.
        launch : str
            The name of the launch file.
        accessory_pkg : str, optional
            The name of the package.
        accessory_launch : str, optional
            The name of the launch file.
    """

    pkg: str
    launch: str
    accessory_pkg: str = None
    accessory_launch: str = None


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
                module_handle, accessory_handle = self.__launch(module)

                # check the module state
                if module is None:
                    print("NO MODULE...")

                
                # take next action from the manual inspector
                prompt = "Next action? \nc -> confirmed \nk -> confirmed but keep inspection view \nr -> restart module)\n>\n "
                action = input(prompt)

                # take action
                if action.lower() == 'c':
                    # close accessory if present
                    if accessory_handle is not None:
                        accessory_handle.shutdown()
                    break

                if action.lower() == 'k':
                    break
                
                # restart the module
                if action.lower() == 'r':
                    if module_handle is not None:
                        module_handle.shutdown()
                    if accessory_handle is not None:
                        accessory_handle.shutdown()
                

    
    def __launch(self, module: Module):
        if module.pkg is None or module.launch is None:
            return None, None


        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        def launch_file(pkg: str, file: str):
            if (None in [pkg, file]):
                return None

            cli_args = [pkg, file]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)

            parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
            parent.start()

            return parent

        return tuple(map(lambda x: launch_file(*x), [(module.pkg, module.launch),
                                                     (module.accessory_pkg, module.accessory_launch)]))
