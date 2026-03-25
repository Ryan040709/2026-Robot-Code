package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.newRobotSubsystems.IndexerSubsystem;

public class Shooter_RunToRPM extends Command {
  ShooterSubsystem shooterSubsystem;
  CommandSwerveDrivetrain drivetrain;
  IndexerSubsystem indexer;

  /** Creates a new Shooter_RunToRPM. */
  public Shooter_RunToRPM(ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrain, IndexerSubsystem indexer) {
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
    shooterSubsystem.RuntoRPMs(drivetrain.getDistanceToTarget(), drivetrain.isFeeding());
    indexer.SetBeltVoltage(7);
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
