package frc.robot.commands.Autopilot;
import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.FieldConstants;
import frc.robot.meth.Alliance;
import frc.robot.meth.Distance;

import static edu.wpi.first.units.Units.*;

public final class AutopilotConstants{
    private static final APConstraints CONSTRAINTS = new APConstraints()
        // TODO: calibrate these values
        .withAcceleration(10)
        .withJerk(5);
        
        

    private static final APProfile PROFILE = new APProfile(CONSTRAINTS)
        // TODO: calibrate these values
        .withErrorXY(Centimeters.of(10))
        .withErrorTheta(Degrees.of(5));
        //.withBeelineRadius(Centimeters.of(8));

    public static final Autopilot AUTOPILOT = new Autopilot(PROFILE);
    
    //How much back to move the target - (MA constants are set for the middle of the game pieces) - currently set as about minus half of the robots length
    public static final double BACKWARDS_OFFSET = -0.3362;

    public static class Targets{
        public static class CoralStation{
            //Coral Station Targets
            public static final Pose2d LEFT_CORAL_STATION = new Pose2d(
                                        FieldConstants.CoralStation.leftCenterFace.getTranslation(),
                                        FieldConstants.CoralStation.leftCenterFace.getRotation()
                                                .plus(Rotation2d.fromDegrees(180)));

                                                
            public static final Pose2d RIGHT_CORAL_STATION = new Pose2d(
                FieldConstants.CoralStation.rightCenterFace.getTranslation(),
                FieldConstants.CoralStation.rightCenterFace.getRotation()
                        .plus(Rotation2d.fromDegrees(180)));
        }
        public static class Reef{
        // This defines a horizontal offset, in meters, from the center of the half targeted reef face.
        // It allows for fine-tuning the robot's final alignment. The offset is always applied
        // from the robot's perspective.
        //
        //   - A POSITIVE value moves the robot to its RIGHT.
        //   - A NEGATIVE value moves the robot to its LEFT.
        //
        // The strategic effect (inward/outward) depends on which side of the reef is targeted:
        //
        // - When targeting the LEFT reef face (e.g., in getLeft... commands):
        //   - POSITIVE (robot moves right) -> moves INWARD toward the reef's centerline.
        //   - NEGATIVE (robot moves left)  -> moves OUTWARD away from the reef's centerline.
        //
        // - When targeting the RIGHT reef face (e.g., in getRight... commands):
        //   - POSITIVE (robot moves right) -> moves OUTWARD away from the reef's centerline.
        //   - NEGATIVE (robot moves left)  -> moves INWARD toward the reef's centerline.
            static final double offsetFromHalfCenter = 0;

            /**
             * Finds the closest Reef face and calculates the target pose for the LEFT side.
             * 
             * @return The target Pose2d, or null if no Reef is found.
             */
            public static Pose2d getLeftReefTarget(Pose2d robotPose) {
                Pose2d closestCenter = Distance.isPointNearLinesSegment(
                        robotPose.getTranslation(),
                        FieldConstants.Reef.centerFaces,
                        FieldConstants.Reef.faceLength,
                        100);

                if (closestCenter != null) {
                    Transform2d toLeftHalf = new Transform2d(0, -(FieldConstants.Reef.faceLength / 4.0 + offsetFromHalfCenter), new Rotation2d());
                    Pose2d targetPoint = closestCenter.transformBy(toLeftHalf);

                    Pose2d finalTarget = new Pose2d(
                            targetPoint.getTranslation(),
                            closestCenter.getRotation().plus(Rotation2d.fromDegrees(180)));

                    return finalTarget;
                }

                return null; // Return null if no Reef was found
            }

            /**
             * Finds the closest Reef face and calculates the target pose for the RIGHT
             * side.
             * 
             * @return The target Pose2d, or null if no Reef is found.
             */
            public static Pose2d getRightReefTarget(Pose2d robotPose) {
                Pose2d closestCenter = Distance.isPointNearLinesSegment(
                        robotPose.getTranslation(),
                        FieldConstants.Reef.centerFaces,
                        FieldConstants.Reef.faceLength,
                        100);

                if (closestCenter != null) {
                    // Use a positive Y value to transform to the right
                    Transform2d toRightHalf = new Transform2d(0, FieldConstants.Reef.faceLength / 4.0 + offsetFromHalfCenter, new Rotation2d());
                    Pose2d targetPoint = closestCenter.transformBy(toRightHalf);

                    Pose2d finalTarget = new Pose2d(
                            targetPoint.getTranslation(),
                            closestCenter.getRotation().plus(Rotation2d.fromDegrees(180)));

                    return finalTarget;
                }

                return null; // Return null if no Reef was found
            }

        }


    }


    
        

    }
