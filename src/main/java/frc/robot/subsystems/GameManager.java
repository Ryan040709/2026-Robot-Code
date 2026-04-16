// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
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

  private XboxController driverController = new XboxController(0);
  private XboxController manipulatorController = new XboxController(1);
  public List<ShiftList> LoseShifts = new ArrayList<>(); // active shifts ASSUMING you lost auto.
  public List<ShiftList> alwaysActiveShifts = new ArrayList<>(); // active shifts ASSUMING you lost auto.

  public ShiftList currentShift = ShiftList.Autonomous;

  private Timer timer = new Timer();
  public PWMSparkFlex shiftLights = new PWMSparkFlex(9);

  public double matchTimer = DriverStation.getMatchTime();
  public double elapsedTime = Timer.getFPGATimestamp();
  Timer isDisabledTimer = new Timer();

  boolean active = false;

  double switchActive = 0;

  public boolean isMatch = true;

  public boolean hasAlliance = false;

  public boolean isTeleop = DriverStation.isTeleop();

  public boolean isActive = true;

  public boolean wonAuto = false;

  public boolean lostAuto = true;

  public static boolean isBlueAlliance = true;
  private int shiftNumber = 0;

  /** Creates a new GameManager. */
  public GameManager() {

    alwaysActiveShifts.add(ShiftList.Autonomous);
    alwaysActiveShifts.add(ShiftList.Transistion);
    LoseShifts.add(ShiftList.One);
    LoseShifts.add(ShiftList.Three);
    alwaysActiveShifts.add(ShiftList.EndGame);
    timer.reset();
    timer.start();
    isDisabledTimer.start();
    SmartDashboard.putBoolean("won Auto", wonAuto);
  }

  @Override
  public void periodic() {
    if (!hasAlliance ||(isDisabledTimer.advanceIfElapsed(4) && DriverStation.isDisabled() )
    ) {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {

        if (alliance.get() == Alliance.Red) {
          isBlueAlliance = false;
          hasAlliance = true;
        }
        if (alliance.get() == Alliance.Blue) {
          isBlueAlliance = true;
          hasAlliance = true;
        }
        SmartDashboard.putBoolean("is blue?", isBlueAlliance);
      }
    }

    //SmartDashboard.putNumber("timer", matchTimer());

    // isTeleop = DriverStation.isTeleop();
    // matchTimer = DriverStation.getMatchTime();
    // elapsedTime = Timer.getFPGATimestamp();

    // determineShift();
    // determineActiveHub();

    // getMatchTime();

  }

  public void setLEDcolor(double setColor) {
    shiftLights.set(setColor);
  }

  public void getMatchTime() {
    if (matchTimer < 0) {
      isMatch = false;
    } else {
      isMatch = true;
    }
  }

  public void wonAuto() {
    wonAuto = true;
    SmartDashboard.putBoolean("won Auto", wonAuto);
  }

  public void switchActiveHubs() {
    if (!active) {
      active = true;
      switchActive = elapsedTime + 25;
    } else if (active) {
      active = false;
      switchActive = elapsedTime + 25;
    }
  }

  public void lostAuto() {

    wonAuto = false;
    SmartDashboard.putBoolean("won Auto", wonAuto);
  }

  public void determineShift() {
    if (isTeleop) {
      if (matchTimer < Constants.GameManager.ShiftEndGame) {
        currentShift = ShiftList.EndGame;
      } else if (matchTimer < Constants.GameManager.ShiftFour) {
        currentShift = ShiftList.Four;
      } else if (matchTimer < Constants.GameManager.ShiftThree) {
        currentShift = ShiftList.Three;
      } else if (matchTimer < Constants.GameManager.ShiftTwo) {
        currentShift = ShiftList.Two;
      } else if (matchTimer < Constants.GameManager.ShiftOne) {
        currentShift = ShiftList.One;
      } else if (matchTimer < Constants.GameManager.ShiftTransistion) {
        currentShift = ShiftList.Transistion;
      }
    }
  }

  public void resetTimer() {
    timer.reset();
  }

  public double matchTimer() {
    return timer.get() + 20;
  }

  public void determineActiveHub() {
    double elapsedTime = matchTimer();
    double activeColor = 0.75; // dark green
    double nonActiveColor = 0.61; // red
    double autoColor = 0.85; // dark blue
    double transitionColor = 0.57;// hot pink
    double warningColor = -.07;// gold strobe
    double endgameColor = 0.91;// violet
    double defaultColor = -0.85;// shot red
    double vibrationPower = 0;

    if (DriverStation.isEnabled()) {
      // auto
      if (elapsedTime <= 17) {
        shiftLights.set(autoColor);
        if (shiftNumber != 1) {
          shiftNumber = 1;
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }

      }
      // warning
      else if (elapsedTime > 17 && elapsedTime <= 20) {
        shiftLights.set(warningColor);
        if (shiftNumber != 2) {
          shiftNumber = 2;
          driverController.setRumble(RumbleType.kBothRumble, vibrationPower);
          manipulatorController.setRumble(RumbleType.kBothRumble, vibrationPower);
        }
      }
      // transistion
      else if (elapsedTime > 20 && elapsedTime <= 27) {
        shiftLights.set(transitionColor);
        if (shiftNumber != 3) {
          shiftNumber = 3;
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }
      }
      // warning
      else if (elapsedTime > 27 && elapsedTime <= 30) {
        shiftLights.set(warningColor);
        if (shiftNumber != 4) {
          shiftNumber = 4;
          driverController.setRumble(RumbleType.kBothRumble, vibrationPower);
          manipulatorController.setRumble(RumbleType.kBothRumble, vibrationPower);
        }
      }
      // phase 1
      else if (elapsedTime > 30 && elapsedTime <= 52) {

        if (wonAuto) {
          shiftLights.set(nonActiveColor);
        } else {
          shiftLights.set(activeColor);
        }
        if (shiftNumber != 5) {
          shiftNumber = 5;
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }
      }
      // warning
      else if (elapsedTime > 52 && elapsedTime <= 55) {
        shiftLights.set(warningColor);
        if (shiftNumber != 6) {
          shiftNumber = 6;
          driverController.setRumble(RumbleType.kBothRumble, vibrationPower);
          manipulatorController.setRumble(RumbleType.kBothRumble, vibrationPower);
        }
      }
      // phase 2
      else if (elapsedTime > 55 && elapsedTime <= 77) {
        if (wonAuto) {
          shiftLights.set(activeColor);
        } else {
          shiftLights.set(nonActiveColor);
        }
        if (shiftNumber != 7) {
          shiftNumber = 7;
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }
      }
      // warning
      else if (elapsedTime > 77 && elapsedTime <= 80) {
        shiftLights.set(warningColor);
        if (shiftNumber != 8) {
          shiftNumber = 8;
          driverController.setRumble(RumbleType.kBothRumble, vibrationPower);
          manipulatorController.setRumble(RumbleType.kBothRumble, vibrationPower);
        }
      }
      // phase 3
      else if (elapsedTime > 80 && elapsedTime <= 102) {
        if (wonAuto) {
          shiftLights.set(nonActiveColor);
        } else {
          shiftLights.set(activeColor);
        }
        if (shiftNumber != 9) {
          shiftNumber = 9;
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }
      }
      // warning
      else if (elapsedTime > 102 && elapsedTime <= 105) {
        shiftLights.set(warningColor);
        if (shiftNumber != 10) {
          shiftNumber = 10;
          driverController.setRumble(RumbleType.kBothRumble, vibrationPower);
          manipulatorController.setRumble(RumbleType.kBothRumble, vibrationPower);
        }
      }
      // phase 4
      else if (elapsedTime > 105 && elapsedTime <= 127) {
        if (wonAuto) {
          shiftLights.set(activeColor);
        } else {
          shiftLights.set(nonActiveColor);
        }
        if (shiftNumber != 11) {
          shiftNumber = 11;
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }
      }
      // warning
      else if (elapsedTime > 127 && elapsedTime <= 130) {
        shiftLights.set(warningColor);
        if (shiftNumber != 12) {
          shiftNumber = 12;
          driverController.setRumble(RumbleType.kBothRumble, vibrationPower);
          manipulatorController.setRumble(RumbleType.kBothRumble, vibrationPower);
        }
      } else if (elapsedTime > 130 && elapsedTime <= 160) {
        shiftLights.set(endgameColor);
        if (shiftNumber != 13) {
          shiftNumber = 13;
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }

      } else {
        shiftLights.set(defaultColor);
        if (shiftNumber != 14) {
          shiftNumber = 14;
          driverController.setRumble(RumbleType.kBothRumble, 0);
          manipulatorController.setRumble(RumbleType.kBothRumble, 0);
        }
      }
    }
  }
}
