from pyfrc.physics import drivetrains


class PhysicsEngine:
    """
    Simulates a 4-wheel robot using Swerve Drive joystick control
    """
    def __init__(self, physics_controller):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                       to communicate simulation effects to
        """
        self.physics_controller = physics_controller
        self.physics_controller.add_device_gyro_channel('navxmxp_spi_4_angle')

    def update_sim(self, hal_data, now, tm_diff):
        """
        Called when the simulation parameters for the program need to be
        updated.
        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        # Invert right side because it is inverted in the mecanum drive method
        lf_motor = hal_data['CAN'][10]['value']
        lr_motor = hal_data['CAN'][15]['value']
        rf_motor = -hal_data['CAN'][20]['value']
        rr_motor = -hal_data['CAN'][25]['value']

        vx, vy, vw = drivetrains.four_motor_swerve_drivetrain(
            lr_motor, rr_motor, lf_motor, rf_motor
        )

        self.physics_controller.vector_drive(vx, vy, vw, tm_diff)
