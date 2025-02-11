package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class IO {
    public static final CommandXboxController driverController = new CommandXboxController(Constants.IO.DRIVER_XBOX_PORT);
    public static final CommandXboxController mechanismController = new CommandXboxController(Constants.IO.MECHANISM_XBOX_PORT);
}
