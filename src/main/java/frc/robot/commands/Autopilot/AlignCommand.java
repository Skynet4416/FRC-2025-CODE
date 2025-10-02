package frc.robot.commands.Autopilot;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.annotation.Target;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;


public class AlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final APTarget target;

    private final SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity);
        
  
    public AlignCommand(APTarget target, CommandSwerveDrivetrain drivetrain) {
      this.target = target;
      this.drivetrain = drivetrain;
      addRequirements(drivetrain);
    }
  
    @Override
    public void initialize() {
      /* no-op */
    }
  

    @Override
    public void execute() {
      ChassisSpeeds fieldRelativeSpeeds = this.drivetrain.getState().Speeds;
      Pose2d pose = drivetrain.getPose();

      Rotation2d correctedRotation = new Rotation2d(-pose.getRotation().getRadians());
      Pose2d correctedPose = new Pose2d(pose.getTranslation(),correctedRotation);

      ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds, pose.getRotation()
    );
      APResult out = AutopilotConstants.AUTOPILOT.calculate(correctedPose, robotRelativeSpeeds, this.target);
  
      double maxTurnSpeed = Math.toRadians(360.0); // 360 degrees per second

      drivetrain.setControl(request
          .withVelocityX(out.vx())
          .withVelocityY(out.vy())
          .withTargetDirection(out.targetAngle())
          .withMaxAbsRotationalRate(maxTurnSpeed)
          .withHeadingPID(2.5, 0, 0.05));
        
      SmartDashboard.putNumber("Target Angle", out.targetAngle().getDegrees());
      SmartDashboard.putNumber("Current Angle", pose.getRotation().getDegrees());
      SmartDashboard.putNumber("Angle Error", out.targetAngle().getDegrees() - pose.getRotation().getDegrees());
      SmartDashboard.putNumber("Autopilot VX (mp/s)", out.vx().magnitude());
      SmartDashboard.putNumber("Autopilot VY (mp/s)", out.vy().magnitude());


    }
  
    @Override
    public boolean isFinished() {
      return AutopilotConstants.AUTOPILOT.atTarget(drivetrain.getPose(),target);
    }
  
    @Override
    public void end(boolean interrupted) {
      ChassisSpeeds zeroSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
      drivetrain.setControl(drivetrain.m_pathApplyRobotSpeeds.withSpeeds(zeroSpeeds));
    }
  }