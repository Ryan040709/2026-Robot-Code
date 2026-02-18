package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class CloseClimber extends Command {
  ClimberSubsystem ClimberSubsystem;
  int counter;

  /** Creates a new ElevatorLowPosision. */
  public CloseClimber(ClimberSubsystem climberSubsystem) {
    this.ClimberSubsystem = climberSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClimberSubsystem.climberServo(Constants.ClimberSubsystem.Climber_Hooks_Closed_Pos);
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
