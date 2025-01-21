// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.chrono.ThaiBuddhistDate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveToPosZ extends Command { //this moves the elevator to a spesfic Z position - clamps values 
  private final ElevatorSubsystem elevatorSubsystem;
  private final double heightInMeters;
  public ElevatorMoveToPosZ(ElevatorSubsystem elevatorSubsystem,double heightInMeters) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.heightInMeters = heightInMeters;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    elevatorSubsystem.setElevetorsDistancenInMeters(heightInMeters);
  }

  @Override
  public void execute() 
  {
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (elevatorSubsystem.getElevetorsDistanceInMeter() < HEIGHT_THRESHOLD){
      return true;
    }
    return false;
  }
}
