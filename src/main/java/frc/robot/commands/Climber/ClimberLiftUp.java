package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberLiftUp extends Command {
  ClimberSubsystem ClimberSubsystem;
  int counter;

  /** Creates a new ElevatorLowPosision. */
  public ClimberLiftUp(ClimberSubsystem climberSubsystem) {
    this.ClimberSubsystem = climberSubsystem;
addRequirements(ClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
ClimberSubsystem.MoveToPosition(Constants.ClimberSubsystem.Climber_Down_Pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ClimberSubsystem.AtGoalPosition(Constants.ClimberSubsystem.Climber_Down_Pos);
  }
}
