package frc.robot.commands.AutoCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.meth.Alliance;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class shaktonomousCommand extends Command {
    private final CommandSwerveDrivetrain drive;
    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController rotationPID;
    private Pose2d robotpose;
    private final Pose2d target;
    private final double offset = 0.05;
    private boolean hasReset = false;

    public shaktonomousCommand(CommandSwerveDrivetrain drive, Pose2d target) {
        this.drive = drive;
        this.target = target;
        xPID = new PIDController(1.5, 0, 0);
        yPID = new PIDController(1.5, 0, 0);
        rotationPID = new PIDController(1, 0, 0);
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
        boolean tv = LimelightHelpers.getTV("");
        if (!tv) {
            return;
        }
        robotpose = LimelightHelpers.getBotPose2d_wpiBlue("");
        if (robotpose == null)
            return;
        // if (robotpose.getX() == 0 && robotpose.getY() == 0) return;
        if (!hasReset) {
            this.drive.resetRotation(Alliance.apply(robotpose.getRotation()));
            hasReset = true;
        }
        
        // Debug output - add these to see what's happening
        SmartDashboard.putNumber("Target X", target.getX());
        SmartDashboard.putNumber("Target Y", target.getY());
        SmartDashboard.putNumber("Robot X", robotpose.getX());
        SmartDashboard.putNumber("Robot Y", robotpose.getY());
        SmartDashboard.putNumber("Robot Rotation", robotpose.getRotation().getDegrees());

        // Calculate errors
        double xError = target.getX() - robotpose.getX();
        double yError = target.getY() - robotpose.getY();

        SmartDashboard.putNumber("X Error", xError);
        SmartDashboard.putNumber("Y Error", yError);

        double xCorrection = MathUtil.clamp(xPID.calculate(robotpose.getX(), target.getX()), -1, 1);
        double yCorrection = MathUtil.clamp(yPID.calculate(robotpose.getY(), target.getY()), -1, 1);
        double rotationCorrection = MathUtil.clamp(rotationPID.calculate(robotpose.getRotation().getDegrees(), target.getRotation().getDegrees()), -1, 1);
        // Debug PID outputs
        SmartDashboard.putNumber("X Correction", xCorrection);
        SmartDashboard.putNumber("Y Correction", yCorrection);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xCorrection,
                yCorrection,
                0,
                robotpose.getRotation());

        drive.setControl(drive.m_pathApplyRobotSpeeds.withSpeeds(chassisSpeeds));
    }

    @Override
    public boolean isFinished() {
        if (robotpose == null)
            return true;

        double xError = Math.abs(robotpose.getX() - target.getX());
        double yError = Math.abs(robotpose.getY() - target.getY());
        return Math.abs(xError) < offset && Math.abs(yError) < offset;
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        drive.setControl(drive.m_pathApplyRobotSpeeds.withSpeeds(zeroSpeeds));
    }
}
