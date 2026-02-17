
package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberTierOne extends Command {
  ClimberSubsystem ClimberSubsystem;
  int counter;

  /** Creates a new ElevatorLowPosision. */
  public ClimberTierOne(ClimberSubsystem climberSubsystem) {
    this.ClimberSubsystem = climberSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (counter == 0) {

    }
    if (ClimberSubsystem.ClimberPast(13)) {
      counter = 1;
      ClimberSubsystem.MoveToPosition(25);
    }
    if (ClimberSubsystem.ClimberPast(32)) {

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ClimberSubsystem.AtGoalPosition(25);
  }
}
