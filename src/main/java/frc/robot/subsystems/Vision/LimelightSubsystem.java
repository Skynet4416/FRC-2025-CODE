// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  private LimelightObserver[] observers;
  private final NetworkTable table;

  public LimelightSubsystem(LimelightObserver[] observers) {
    this.observers = observers;
    this.table = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public void periodic() {
    boolean targetVisible = table.getEntry("tv").getDouble(0) == 1;
    if (targetVisible){
      updateLimelightData(targetVisible);
    }
  }

  public void updateLimelightData(boolean targetVisible) {
    double[] botPose = table.getEntry("botpose").getDoubleArray(new double[6]);
    for (LimelightObserver observer : observers) {
        observer.onLimelightDataUpdate(targetVisible, botPose);
    }
  }
}