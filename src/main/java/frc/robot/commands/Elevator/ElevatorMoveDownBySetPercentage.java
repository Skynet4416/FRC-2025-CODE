// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.Subsystems.Elevator;

public class ElevatorMoveDownBySetPercentage extends Command {

    private final ElevatorSubsystem elevatorSubsystem;

    public ElevatorMoveDownBySetPercentage(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    public void initialize() {
        elevatorSubsystem.setPercentage(Elevator.Controls.ELEVATOR_PERCENTAGE);
    }


    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setElevatorDistanceInMeters(elevatorSubsystem.getElevatorDistanceInMeter());
    }


}
