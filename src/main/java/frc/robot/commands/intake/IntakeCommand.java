package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {

  boolean intakeValue = false;

  /** Creates a new intake. */
  public IntakeCommand() {
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
intakeValue = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

 SmartDashboard.putBoolean("intake", intakeValue);

  }

  @Override
  public void end(boolean interrupted) {
    //intake off
    intakeValue = false; 
    SmartDashboard.putBoolean("intake", intakeValue);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
