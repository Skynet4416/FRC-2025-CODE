package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class moveToPoint extends Command {
    private final CommandSwerveDrivetrain drive;
    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController rotationPID;
    private Pose2d robotpose;
    private final Pose2d target;
    private final double tolerance = 0.05;

    public moveToPoint(CommandSwerveDrivetrain drive, Pose2d target) {
        this.drive = drive;
        this.target = target;
        xPID = drive.getXPathController();
        yPID = drive.getYPathController();
        rotationPID = drive.getRotationPathController();
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        xPID.reset();
        yPID.reset();
        rotationPID.reset();

    }

    @Override
    public void execute() {
        robotpose = drive.getPose();

        double xCorrection = MathUtil.clamp(xPID.calculate(robotpose.getX(), target.getX()), -1, 1);
        double yCorrection = MathUtil.clamp(yPID.calculate(robotpose.getY(), target.getY()), -1, 1);
        double rotationCorrection = MathUtil.clamp(
                rotationPID.calculate(robotpose.getRotation().getDegrees(), target.getRotation().getDegrees()), -1, 1);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xCorrection,
                yCorrection,
                rotationCorrection,
                robotpose.getRotation());

        drive.setControl(drive.m_pathApplyRobotSpeeds.withSpeeds(chassisSpeeds));
    }

    @Override
    public boolean isFinished() {
        return robotpose.getTranslation().getDistance(target.getTranslation()) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        drive.setControl(drive.m_pathApplyRobotSpeeds.withSpeeds(zeroSpeeds));
    }
}
