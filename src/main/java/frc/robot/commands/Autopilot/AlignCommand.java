package frc.robot.commands.Autopilot;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import java.lang.annotation.Target;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;


public class AlignCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final APTarget target;

    private final SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle()
        .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withHeadingPID(0.5, 0, 0.05); //not tuned
        
  
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

      ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        fieldRelativeSpeeds, pose.getRotation()
    );
      APResult out = AutopilotConstants.AUTOPILOT.calculate(pose, robotRelativeSpeeds, this.target);
  
      double maxTurnSpeed = Math.toRadians(360.0); // 360 degrees per second

      drivetrain.setControl(request
          .withVelocityX(out.vx())
          .withVelocityY(out.vy())
          .withTargetDirection(out.targetAngle())
          .withMaxAbsRotationalRate(maxTurnSpeed));
        
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