package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterStop extends Command {
  ShooterSubsystem shooterSubsystem;
  CommandSwerveDrivetrain drivetrain;
  IndexerSubsystem indexer;

  /** Creates a new Shooter_RunToRPM. */
  public ShooterStop(ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrain, IndexerSubsystem indexer) {
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrain = drivetrain;
    this.indexer = indexer;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.SetShooterRPMS(0);
    indexer.SetBeltVoltage(7); // TODO: still needs to be tuned to real robot?
    indexer.SetIndexSpeed(0.5);
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
