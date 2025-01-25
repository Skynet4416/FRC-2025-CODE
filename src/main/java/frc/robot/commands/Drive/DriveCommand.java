package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final DoubleSupplier speedModSupplier;

    public DriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, DoubleSupplier speedModSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.speedModSupplier = speedModSupplier;
        addRequirements(driveSubsystem);
    }

    public double accountForDrift(DoubleSupplier supplier) {
        return Math.max(0,Math.abs(supplier.getAsDouble()) - Constants.OIConstants.kXboxControllerDrift) * Math.signum(supplier.getAsDouble());
    }

    @Override
    public void execute() {
        double speedSlower = 1 - this.speedModSupplier.getAsDouble()*0.9;
        double xSpeed = accountForDrift(xSupplier) * speedSlower;
        double ySpeed = accountForDrift(ySupplier) * speedSlower;
        double rotSpeed = accountForDrift(rotationSupplier);

        driveSubsystem.getDrivetrain().setControl(
            driveSubsystem.getDriveRequest()
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotSpeed)
        );
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.getDrivetrain().setControl(
            driveSubsystem.getDriveRequest()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
