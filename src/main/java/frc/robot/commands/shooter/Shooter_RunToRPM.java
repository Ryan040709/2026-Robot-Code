package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter;

public class Shooter_RunToRPM extends Command {
  Shooter s_ShooterWheel; 
  /** Creates a new Shooter_RunToRPM. */
  public Shooter_RunToRPM(Shooter s_ShooterWheel) {
    this.s_ShooterWheel = s_ShooterWheel;
    addRequirements(s_ShooterWheel);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_ShooterWheel.RuntoRPMs();
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
