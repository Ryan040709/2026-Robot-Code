// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkFlex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GameManager extends SubsystemBase {

  public enum ShiftList {
    Autonomous, // active for both
    Transistion, // active for both
    One, // alternates
    Two, // alternates
    Three, // alternates
    Four, // alternates
    EndGame // active for both
  }

  public List<ShiftList> LoseShifts = new ArrayList<>(); // active shifts ASSUMING you lost auto.
  public List<ShiftList> alwaysActiveShifts = new ArrayList<>(); // active shifts ASSUMING you lost auto.

  public ShiftList currentShift = ShiftList.Autonomous;

  public PWMSparkFlex shiftLights = new PWMSparkFlex(0);

  public double matchTimer = DriverStation.getMatchTime();
  public double elapsedTime = Timer.getFPGATimestamp();

  boolean active = false;

  double switchActive = 0;

  public boolean isMatch = true;

  public boolean isTeleop = DriverStation.isTeleop();

  public boolean isActive = true;

  public boolean wonAuto = false;

  public boolean lostAuto = true;

  public static boolean isBlueAlliance = true;

  /** Creates a new GameManager. */
  public GameManager() {
    SmartDashboard.putData("Win Auto?", runOnce(() -> wonAuto()));
    SmartDashboard.putData("Lost Auto?", runOnce(() -> lostAuto()));

    alwaysActiveShifts.add(ShiftList.Autonomous);
    alwaysActiveShifts.add(ShiftList.Transistion);
    LoseShifts.add(ShiftList.One);
    LoseShifts.add(ShiftList.Three);
    alwaysActiveShifts.add(ShiftList.EndGame);

  }

  @Override
  public void periodic() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        isBlueAlliance = false;
      }
      if (alliance.get() == Alliance.Blue) {
        isBlueAlliance = true;
      }
    } else {
      isBlueAlliance = false;
    }

    isTeleop = DriverStation.isTeleop();
    matchTimer = DriverStation.getMatchTime();
    elapsedTime = Timer.getFPGATimestamp();

    determineShift();
    determineActiveHub();

    SmartDashboard.putNumber("GameManager Timer", elapsedTime);

    SmartDashboard.putString("current shift", currentShift.toString());

    SmartDashboard.putNumber("match timer", matchTimer);
    SmartDashboard.putBoolean("is teleop", isTeleop);
    SmartDashboard.putBoolean("win auto?", wonAuto);

    SmartDashboard.putBoolean("is active?", isActive);
    SmartDashboard.putBoolean("is in a match?", isMatch);

    getMatchTime();
  }

  public void getMatchTime() {
    if (matchTimer < 0) {
      isMatch = false;
    } else {
      isMatch = true;
    }
  }

  public void wonAuto() {
    if (!wonAuto) {
      wonAuto = true;
    } else {
      wonAuto = false;
    }
  }

  public void switchActiveHubs() {
    if (!active) {
      shiftLights.set(0.5);
      active = true;
      switchActive = elapsedTime + 25;
    } else if (active) {
      shiftLights.set(0.5);
      active = false;
      switchActive = elapsedTime + 25;
    }

    SmartDashboard.putBoolean("teleop active", active);
    SmartDashboard.putNumber("time till swicth", switchActive);
  }

  public void lostAuto() {
    if (!wonAuto) {
      lostAuto = true;
    } else {
      lostAuto = false;
    }
  }

  public void determineShift() {
    if (isTeleop) {
      if (matchTimer < Constants.GameManager.shiftEndGame) {
        currentShift = ShiftList.EndGame;
      } else if (matchTimer < Constants.GameManager.shiftFour) {
        currentShift = ShiftList.Four;
      } else if (matchTimer < Constants.GameManager.shiftThree) {
        currentShift = ShiftList.Three;
      } else if (matchTimer < Constants.GameManager.shiftTwo) {
        currentShift = ShiftList.Two;
      } else if (matchTimer < Constants.GameManager.shiftOne) {
        currentShift = ShiftList.One;
      } else if (matchTimer < Constants.GameManager.shiftTransistion) {
        currentShift = ShiftList.Transistion;
      }
    }
  }

  public void determineActiveHub() {
    if (isMatch) {
      if (wonAuto) {
        if (LoseShifts.contains(currentShift)) {
          // not active
          // shiftLights.set(0.61);
          isActive = false;
        } else {
          // active
          // shiftLights.set(0.77);
          isActive = true;
        }
      } else {
        if (LoseShifts.contains(currentShift) || alwaysActiveShifts.contains(currentShift)) {
          // active
          // shiftLights.set(0.77);
          isActive = true;
        } else {
          // not active
          // shiftLights.set(0.61);
          isActive = false;
        }
      }
    } else if (elapsedTime > switchActive) {
      // switchActiveHubs();
      if (!active) {
        shiftLights.set(0.77);
      } else {
        shiftLights.set(0.61);
      }
    }
  }
}
