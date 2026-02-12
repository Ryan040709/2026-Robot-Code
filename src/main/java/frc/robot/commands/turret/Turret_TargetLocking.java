package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurretSubsystem;

public class Turret_TargetLocking extends Command {

  boolean intakeValue = false;

  boolean turretLocking = true;

  TurretSubsystem TurretSubsystem;

  /** Creates a new Turret_TargetLocking. */
  public Turret_TargetLocking(TurretSubsystem turretSubsystem) {
    this.TurretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeValue = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("shooter Tracking");
    TurretSubsystem.turretToggle();

    SmartDashboard.putBoolean("turret", intakeValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intakeValue = false;
    SmartDashboard.putBoolean("turret", intakeValue);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
