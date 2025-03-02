// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.LimelightHelpers.PoseEstimate;

public class LimelightSubsystem extends SubsystemBase {

    private final LimelightObserver[] observers;

    public LimelightSubsystem(LimelightObserver[] observers) {
        this.observers = observers;
    }

    @Override
    public void periodic() {
        updateLimelightData();
    }

    public void updateLimelightData() {
        boolean useMegaTag2 = true; // set to false to use MegaTag1
        boolean doRejectUpdate = false;

        Vector<N3> stdDiviation = null;
        LimelightHelpers.PoseEstimate poseEstimate = null;
        if (useMegaTag2 == false) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
            stdDiviation = VecBuilder.fill(.5, .5, 9999999);
        } else if (useMegaTag2 == true) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
            stdDiviation = VecBuilder.fill(.7, .7, 9999999);
        }

        if (poseEstimate != null && poseEstimate.rawFiducials.length > 0 && poseEstimate.rawFiducials[0].ambiguity > 0.5 && poseEstimate.rawFiducials[0].distToCamera > 4) {
            doRejectUpdate = true;
        }

        if (poseEstimate != null && poseEstimate.tagCount > 0 && !doRejectUpdate) {
            for (LimelightObserver observer : observers) {
                observer.onLimelightDataUpdate(poseEstimate, stdDiviation);
            }
            SmartDashboard.putBoolean("VISION REJECTED", false);

        } else {
            SmartDashboard.putBoolean("VISION REJECTED", true);
        }

        if(poseEstimate != null){SmartDashboard.putNumberArray("LL pose", new Double[]{
                poseEstimate.pose.getX(), poseEstimate.pose.getY(), poseEstimate.pose.getRotation().getDegrees()
        });}
    }

}
