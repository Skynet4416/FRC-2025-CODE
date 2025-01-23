// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

//
public class ElevatorMoveUpBySetPercentage extends Command {
    private final ElevatorSubsystem elevatorSubsystem;

  public ElevatorMoveUpBySetPercentage(ElevatorSubsystem elevatorSubsystem) { //Move up slowly for user control adjusment 
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setElevetorsDistancenInMeters(elevatorSubsystem.getElevetorsDistanceInMeter() * UP_ADJUSMENT_MOVEMENT_IN_PERCENTAGE); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setElevetorsDistancenInMeters(elevator.getElevetorsDistanceInMeter());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
