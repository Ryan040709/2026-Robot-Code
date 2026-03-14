package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.hoodSubsystem;

public class OverrideHoodPosition extends Command {
  hoodSubsystem hoodSubsystem;

  /** Creates a new Shooter_RunToPosition. */
  public OverrideHoodPosition(hoodSubsystem hoodSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    addRequirements(hoodSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // System.out.println("hood Run");
    hoodSubsystem.moveHoodToPosition(0);
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
