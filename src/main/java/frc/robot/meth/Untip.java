package frc.robot.meth;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Untip {
    public static Translation2d calcualteVelocity(double robotRelativeX, double robotReletiveY, Pigeon2 pigeon){
        double speedX = robotRelativeX;
        double speedY = robotReletiveY;
        double anglePitch = pigeon.getPitch().getValueAsDouble();
        double angleRoll = pigeon.getRoll().getValueAsDouble();

        SmartDashboard.putNumber("robotPitch" , anglePitch);
        SmartDashboard.putNumber("robotRoll" , angleRoll);

        if(anglePitch <= 45.0 && anglePitch >= -45.0 ){
            speedX += (anglePitch/45) * 0.25;
        }
        if(angleRoll <= 45.0 && angleRoll >= -45.0 ){
            speedY += (angleRoll/45) * 0.25;
        }
        return new Translation2d(speedX,speedY);
    }
}
