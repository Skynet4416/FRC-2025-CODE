// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.time.chrono.ThaiBuddhistDate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMoveToReefPosY extends Command { //this moves the elevator to a spesfic Y position for REEF 
  private final ElevatorSubsystem elevatorSubsystem;
  public ElevatorMoveToReefPosY(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    elevatorSubsystem.StopMotors(); //assuming won't fall - can be removed
  }

  @Override
  public void execute() 
  {
    elevatorSubsystem.setElevetorsDistancenInMeters(REEF_POS); //add constant -- calibrated 
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
