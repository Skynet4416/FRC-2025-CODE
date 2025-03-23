package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import choreo.Choreo;
import choreo.trajectory.TrajectorySample;

public class ShakBarzLib {

    public static double getClosestPoint(String pathName, Pose2d pose) {
        double min = 99999999;
        double minTimeStemp = 0;
        for (TrajectorySample sample : Choreo.loadTrajectory(pathName).get().samples()) {

            if (sample.getPose().getTranslation().getDistance(pose.getTranslation()) < min) {
                minTimeStemp = sample.getTimestamp();
                min = sample.getPose().getTranslation().getDistance(pose.getTranslation());
            }

        }
        return minTimeStemp;
    }

}