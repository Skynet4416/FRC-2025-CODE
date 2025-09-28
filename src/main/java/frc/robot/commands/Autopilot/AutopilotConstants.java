package frc.robot.commands.Autopilot;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;

import static edu.wpi.first.units.Units.*;

public final class AutopilotConstants{
    private static final APConstraints CONSTRAINTS = new APConstraints()
        // TODO: calibrate these values
        .withAcceleration(10)
        .withJerk(2.0);

    private static final APProfile PROFILE = new APProfile(CONSTRAINTS)
        // TODO: calibrate these values
        .withErrorXY(Centimeters.of(3))
        .withErrorTheta(Degrees.of(5));
        //.withBeelineRadius(Centimeters.of(8));

    public static final Autopilot AUTOPILOT = new Autopilot(PROFILE);

    //TODO: define the needed targets
    public static final APTarget target = new APTarget(FieldConstants.CoralStation.leftCenterFace)
    .withEntryAngle(Rotation2d.fromDegrees(90));
    }
