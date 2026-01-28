  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {

  boolean intakeValue = false;

  /** Creates a new intake. */
  public IntakeCommand() {
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

 SmartDashboard.putBoolean("intake", intakeValue);

  }

//@Override
    // public void periodic() {

    // }

  // Called once the command ends or is interrupted.
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
