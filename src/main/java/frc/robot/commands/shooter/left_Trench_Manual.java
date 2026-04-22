package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class left_Trench_Manual extends Command {
  ShooterSubsystem shooterSubsystem;
  CommandSwerveDrivetrain drivetrain;
  TurretSubsystem turretSubsystem;
  double manualRPMS;

  /** Creates a new Shooter_RunToRPM. */
  public left_Trench_Manual(ShooterSubsystem shooterSubsystem, CommandSwerveDrivetrain drivetrain, TurretSubsystem turretSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.drivetrain = drivetrain;
    this.turretSubsystem = turretSubsystem;
    addRequirements(shooterSubsystem, turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // SmartDashboard.putNumber("manual RPMS", 0);
   // manualRPMS = SmartDashboard.getNumber("manual RPMS", 0);
    shooterSubsystem.SetShooterRPMS(60);
    turretSubsystem.maunaulTurretPosition(-70);
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
