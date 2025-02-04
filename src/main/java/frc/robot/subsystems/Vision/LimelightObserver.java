// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N3;

public interface LimelightObserver {

    void onLimelightDataUpdate(LimelightHelpers.PoseEstimate pose, Vector<N3> stdDiviation);
}
