// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroTurret extends Command {
  TurretSubsystem turretSubsystem;
  private int counter;
  private boolean turretIsZeroed;
  /** Creates a new ZeroTurret. */
  public ZeroTurret( TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    turretIsZeroed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        if (counter == 0) {
            if (turretSubsystem.zeroingSensor.get() == true) {
                turretSubsystem.setTurretPower(.04);
                System.out.println("moving to sensor");
            } else if (turretSubsystem.zeroingSensor.get() == false) {
                counter = 1;
               turretSubsystem.setTurretPower(.03);
                
                System.out.println("at sensor");
            }
        } else {
            if (turretSubsystem.zeroingSensor.get() == true) {
                turretSubsystem.setTurretPower(0);
                turretSubsystem.zeroTurretPosition(4.071777);
                turretIsZeroed = true;
                
                System.out.println("zeroing complete");
            } else if (turretSubsystem.zeroingSensor.get() == false) {
                turretSubsystem.setTurretPower(.03);
                
                System.out.println("moving away from sensor");
            }
        }
        SmartDashboard.putNumber("counter", counter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turretIsZeroed;
  }
}
