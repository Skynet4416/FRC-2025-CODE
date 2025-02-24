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

public class Forwards extends Command {
    private final CommandSwerveDrivetrain drive;
    private boolean hasReset = false;
    private Pose2d robotpose;

    public Forwards(CommandSwerveDrivetrain drive) {
        this.drive = drive;
        addRequirements(drive);

    }

    @Override
    public void initialize() {   

     }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(2.0, 0.0, 0.0);
        drive.setControl(drive.m_pathApplyRobotSpeeds.withSpeeds(chassisSpeeds));
        // boolean tv = LimelightHelpers.getTV("");
        // if (!tv) {
        //     return;
        // }
        // robotpose = LimelightHelpers.getBotPose2d_wpiBlue("");
        // if (robotpose == null)
        //     return;
        // // if (robotpose.getX() == 0 && robotpose.getY() == 0) return;
        // if (!hasReset) {
        //     this.drive.resetRotation(Alliance.apply(robotpose.getRotation()));
        //     hasReset = true;
        // }
        


    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        // drive.setControl(drive.m_pathApplyRobotSpeeds.withSpeeds(zeroSpeeds));
    }
}
