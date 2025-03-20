// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralManipulatorConstants;
import frc.robot.subsystems.AlgaeManipulatorSubsystem;
import frc.robot.subsystems.AlgaeManipulatorSubsystem.AP;
import frc.robot.subsystems.CoralManipulatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.SP;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TransitModeCommand extends Command {

  ElevatorSubsystem elevator;
  CoralManipulatorSubsystem coral;
  AlgaeManipulatorSubsystem algae;

  /** Creates a new TransitModeCommand. */
  public TransitModeCommand(ElevatorSubsystem elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    this.elevator = elevator;
    this.coral = null;
    this.algae = null;
  }

  /** Creates a new TransitModeCommand. */
  public TransitModeCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coral) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, coral);

    this.elevator = elevator;
    this.coral = coral;
    this.algae = null;
  }

  /** Creates a new TransitModeCommand. */
  public TransitModeCommand(ElevatorSubsystem elevator, CoralManipulatorSubsystem coral, AlgaeManipulatorSubsystem algae ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator, coral, algae);

    this.elevator = elevator;
    this.coral = coral;
    this.algae = algae;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator != null)
      elevator.moveToScoringPosition(SP.min); 
    if (coral != null)
      coral.movePivotTo(CoralManipulatorConstants.transitPivotPosition);
    if (algae != null)
      algae.setDesiredPivotHeight(AP.transit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
