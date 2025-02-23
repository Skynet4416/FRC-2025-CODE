package frc.robot.commands.Autos;


import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.LimelightHelpers;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
public class CenterByTag extends Command {
    boolean dididoityesno = false;
    CommandSwerveDrivetrain drive;
    public CenterByTag(CommandSwerveDrivetrain drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
    }
    public void execute() {
        boolean tv = LimelightHelpers.getTV("");
        if (!tv) {
            return; 
        }
        
        double tagID = LimelightHelpers.getFiducialID("");
        var field = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField(); 
        var tagPoseOpt = field.getTagPose((int) tagID); 
        

        if (tagPoseOpt.isEmpty()) {
            return;
        }
        
        Pose3d tagPose = tagPoseOpt.get();
        double tagYaw = tagPose.getRotation().getZ(); 
        double tx = LimelightHelpers.getTX(""); 
        
        double gyroRotation = drive.getGyroRotationInDegrees(); 
        
        DriverStation.getAlliance().ifPresent(allianceColor -> {
            Rotation2d newRotation = allianceColor == Alliance.Red
                ? Rotation2d.fromDegrees(degreesFixer(gyroRotation - tagYaw - tx - 180))
                : Rotation2d.fromDegrees(degreesFixer(gyroRotation - tagYaw - tx));
            
            drive.setOperatorPerspectiveForward(newRotation);
            
            dididoityesno = true;
        });
    }
    
    

    @Override
    public boolean isFinished() { 
        return dididoityesno;
    }

    @Override
    public void end(boolean interrupted) {
    }

    public double degreesFixer(double num){
        if(num<0){
            return 360+num;
        }
        return num;
    }
}
