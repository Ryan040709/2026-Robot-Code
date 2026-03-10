// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightHelpers.RawDetection;

/** Add your docs here. */
public class GamePieceDetection extends SubsystemBase {
  double gamePieceTX;
  // distance away from wall is already set to .762 before you add anymore
  double distanceAwayBeforeSlowDown = .3;
  PIDController rotationPID = new PIDController(.025, 0, 0);
  double aspect;
  // aspect Threshold was 1
  double aspectThreshold = 0;

  public GamePieceDetection() {

  }

  public void resetPID() {
    rotationPID.reset();
  }

  public boolean gamePieceDetected() {
    return LimelightHelpers.getTV("limelight-tags");
  }

  // is there not a better method for calculating speeds???
  public double computeMaxDriveSpeeds(Pose2d robotpose) {

    if (robotpose.getX() < .76 + distanceAwayBeforeSlowDown || robotpose.getX() > 16.76 - distanceAwayBeforeSlowDown
        || robotpose.getY() < .76 + distanceAwayBeforeSlowDown || robotpose.getY() > 7.29 - distanceAwayBeforeSlowDown
        || robotpose.getY() < (-.72 * robotpose.getX()) + (2.193 + (1.75 * distanceAwayBeforeSlowDown))
        || robotpose.getY() > (.72 * robotpose.getX()) + (5.85 - (1.75 * distanceAwayBeforeSlowDown))
        || robotpose.getY() > (-.72 * robotpose.getX()) + (18.47 - (1.75 * distanceAwayBeforeSlowDown))
        || robotpose.getY() < (.72 * robotpose.getX()) - (10.42 + (1.75 * distanceAwayBeforeSlowDown))) {
      return .25;
    } else {
      return 1;
    }
  }

  public ChassisSpeeds driveSpeeds(Pose2d initialrobotpose, Pose2d robotpose) {
    double rotationSpeed = rotationPID.calculate(gamePieceTX, 0);
    double maxDriveSpeed = computeMaxDriveSpeeds(robotpose);

    findAspect();

    if (aspect > aspectThreshold) {
      return new ChassisSpeeds(0, (2.5 * maxDriveSpeed), rotationSpeed * 3);
    } else {

      double direction = Math.signum(4 - initialrobotpose.getY())
          * Math.signum(initialrobotpose.getRotation().getDegrees() - 0);
      return new ChassisSpeeds(2 * direction * maxDriveSpeed, 0, rotationSpeed * 2);
    }
  }

  public void findAspect() {

    double[] pose = LimelightHelpers.getLimelightNTDoubleArray("limelight-intake", "tcornxy");

    if (pose.length >= 8) {
      double corner0_X = pose[0];
      double corner0_y = pose[1];
      double corner1_X = pose[4];
      double corner1_y = pose[5];
      SmartDashboard.putNumber("Corner 0 X", corner0_X);
      SmartDashboard.putNumber("Corner 0 Y", corner0_y);
      SmartDashboard.putNumber("Corner 1 X", corner1_X);
      SmartDashboard.putNumber("Corner 1 y", corner1_y);
      double cornerXdistance = corner1_X - corner0_X;
      double cornerYdistance = corner1_y - corner0_y;
      SmartDashboard.putNumber("corner x distance", cornerXdistance);
      SmartDashboard.putNumber("corner Y distance", cornerYdistance);
      aspect = cornerXdistance / cornerYdistance;
      SmartDashboard.putNumber("corner Aspect", aspect);
    }

  }

  @Override
  public void periodic() {
    gamePieceTX = LimelightHelpers.getTX("limelight-tags");
    SmartDashboard.putNumber("Game Piece TX", gamePieceTX);

  }
}