// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.GamePieceDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class drivetrain_DriveAtFuel extends Command {
  GamePieceDetection gamePieceDetection;
  CommandSwerveDrivetrain drivetrain;
  long shooterStartTime;
  Pose2d initialrobotPose;

  /** Creates a new DriveAtCoral. */
  public drivetrain_DriveAtFuel(GamePieceDetection gamePieceDetection, CommandSwerveDrivetrain drivetrain) {
    this.gamePieceDetection = gamePieceDetection;
    this.drivetrain = drivetrain;
    addRequirements(gamePieceDetection, drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gamePieceDetection.resetPID();
    shooterStartTime = -1;
    initialrobotPose = drivetrain.getPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // the boolean below does nothing?
    if (shooterStartTime == -1) {

      shooterStartTime = System.currentTimeMillis();
    }
    if (gamePieceDetection.gamePieceDetected()) {
      drivetrain.driveRobotRelative(gamePieceDetection.driveSpeeds(initialrobotPose, drivetrain.getPose()));
    } else {
      drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  // ironically I think this code was pulled from crescendo, then put into a
  // stacking game, then a shooter game... again.
  public boolean isFinished() {
    return false; // shooterStartTime != -1 && (System.currentTimeMillis() - shooterStartTime) >
                  // 1500;
  }
}
