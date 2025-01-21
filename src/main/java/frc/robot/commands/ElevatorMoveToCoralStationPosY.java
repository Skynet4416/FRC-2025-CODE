// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.chrono.ThaiBuddhistDate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveToCoralStationPosY extends Command { //this moves the elevator to a spesfic Y position for CORAL STATION input
  private final ElevatorSubsystem elevatorSubsystem;
  public ElevatorMoveToCoralStationPosY(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    elevatorSubsystem.StopMotors(); //assuming won't fall - can be removed
    elevatorSubsystem.setElevetorsDistancenInMeters(CORAL_STATION_POS); //add constant -- calibrated

  }

  @Override
  public void execute() 
  {
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
