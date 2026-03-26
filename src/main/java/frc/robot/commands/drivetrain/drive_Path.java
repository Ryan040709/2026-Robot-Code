package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class drive_Path extends Command {
  CommandSwerveDrivetrain drivetrain;
  Command delegate;

  public drive_Path(CommandSwerveDrivetrain drivetrain){
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
