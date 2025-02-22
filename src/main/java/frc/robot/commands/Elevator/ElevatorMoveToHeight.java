// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorMoveToHeight extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final double elevatorSetPointInMeters;

    public ElevatorMoveToHeight(ElevatorSubsystem elevatorSubsystem, double heightInMeters) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.elevatorSetPointInMeters = heightInMeters;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void execute() {
        elevatorSubsystem.setPercentage(Elevator.Controls.ELEVATOR_PERCENTAGE * (elevatorSubsystem.getElevatorDistanceInMeter() > elevatorSetPointInMeters ? -1 : 1));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSubsystem.getElevatorDistanceInMeter() - this.elevatorSetPointInMeters) < Elevator.Controls.HEIGHT_THRESHOLD_IN_METERS;
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setPercentage(0);
    }
}
