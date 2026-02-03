// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberLower extends Command {
  ClimberSubsystem ClimberSubsystem;
  int counter;
  /** Creates a new ElevatorLowPosision. */
  public ClimberLower(ClimberSubsystem climberSubsystem) {
    this.ClimberSubsystem = climberSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter =0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(counter == 0){

    }
    if(ClimberSubsystem.ClimberPast(13)){
      counter =1;
    ClimberSubsystem.MoveToPosition(0);
    }
    if(ClimberSubsystem.ClimberPast(32)){

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ClimberSubsystem.AtGoalPosition(0);
  }
}
