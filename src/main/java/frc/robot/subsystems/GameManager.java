// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GameManager extends SubsystemBase {

  
  public enum CurrentShift{
    Autonomous, //active for both
    Transistion, // active for both
    One, //alternates
    Two, //alternates
    Three, //alternates
    Four, //alternates
    EndGame //active for both
  }

  public PWMSparkFlex shiftLights = new PWMSparkFlex(1);

  public double matchTimer = DriverStation.getMatchTime();

  //public String currentShift = "auto";


  public boolean isTeleop = DriverStation.isTeleop();

  public boolean wonAuto = false;

  public boolean lostAuto = true;

  /** Creates a new GameManager. */
  public GameManager() {
    SmartDashboard.putData("Win Auto?", runOnce(() -> wonAuto()));
    SmartDashboard.putData("Lost Auto?", runOnce(() -> lostAuto()));
  }

  @Override
  public void periodic() {
    isTeleop = DriverStation.isTeleop();
    matchTimer = DriverStation.getMatchTime();

    SmartDashboard.putNumber("match timer", matchTimer);
    SmartDashboard.putBoolean("is teleop", isTeleop);
  }

  public void wonAuto() {
    if (!wonAuto) {
      wonAuto = true;
    } else {
      wonAuto = false;
    }
  }

  public void lostAuto() {
    if (!wonAuto) {
      lostAuto = true;
    } else {
      lostAuto = false;
    }
  }

  public void determineShift() {
    
  }
}
