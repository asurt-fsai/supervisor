import roslaunch


class Inspector():


    def __init__(self, launch_file_pairs):
        self.launch_file_pairs: list = launch_file_pairs


    def manual_inspect(self):
        for launche_file_pair in self.launch_file_pairs:
            # until passed
            while True:
                # launch the module, and get handles
                module, accessory = self.__launch(launche_file_pair)
                
                # take next action from the manual inspector
                action = input("Next action? \nc -> confirmed \nk -> confirmed but keep inspection view \nr -> restart module)\n> ")

                # take action
                if action.lower() == 'c':
                    # close any accessory
                    if accessory is not None:
                        accessory.shutdown()
                    break

                if action.lower() == 'k':
                    break
                
                if action.lower() == 'r':
                    if module is not None:
                        module.shutdown()
                    if accessory is not None:
                        accessory.shutdown()
                

    
    def __launch(self, launch_file_pair):
        if len(launch_file_pair) < 1:
            return
        
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args = ['mrpython_pcl', 'lidar.launch']
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)

        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)

        return parent, None
