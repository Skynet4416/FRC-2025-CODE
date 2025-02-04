// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

    private LimelightObserver[] observers;

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
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
            stdDiviation = VecBuilder.fill(.5, .5, 9999999);
        } else if (useMegaTag2 == true) {
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            stdDiviation = VecBuilder.fill(.7, .7, 9999999);
        }

        if (poseEstimate.rawFiducials.length > 0 && poseEstimate.rawFiducials[0].ambiguity > 0.5 && poseEstimate.rawFiducials[0].distToCamera > 4) {
            doRejectUpdate = true;
        }

        if (poseEstimate.tagCount > 0 && !doRejectUpdate) {
            for (LimelightObserver observer : observers) {
                observer.onLimelightDataUpdate(poseEstimate, stdDiviation);
            }
            SmartDashboard.putBoolean("VISION REJECTED", false);

        } else {
            SmartDashboard.putBoolean("VISION REJECTED", true);
        }

        SmartDashboard.putNumberArray("LL pose", new Double[]{
                poseEstimate.pose.getX(), poseEstimate.pose.getY(), poseEstimate.pose.getRotation().getDegrees()
        });
    }

}
