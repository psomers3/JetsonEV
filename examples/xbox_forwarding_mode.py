from JetsonEV import JetsonEV
import time


if __name__ == '__main__':
    driver_input = [0]  # create value to hold current desired input. Do inside a list so python passes it by reference

    # define our function to handle signals from the xbox controller. This one simply stores the value in a 'global'
    # variable
    def update_driver_input(axis):
        """
        :param axis: will be an axis object from the xbox360controller module
        """
        driver_input[0] = axis.value

    # create the car
    car = JetsonEV(mode=JetsonEV.xbox_forwarding_mode, max_speed_limit=0.25)

    # assign our function to the right trigger of the xbox controller
    car.set_trigger_r_func(update_driver_input)

    simulation_step_length = 20  # arbitrary number for this example

    # Do some "fixed" time step operations like some kind of control loop just grabbing the latest user input as it is
    # required.
    for i in range(simulation_step_length):
        current_input = driver_input[0]
        print('Calculating my control input here from a driver input of ', current_input)
        print('Applying my control input...')
        print('end of loop iteration ', i, '\n')
        time.sleep(0.2)

    car.shutdown()
    exit()
