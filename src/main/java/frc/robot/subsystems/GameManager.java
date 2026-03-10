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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

 private final SendableChooser<Command> winChooser;

  public ShiftList currentShift = ShiftList.Autonomous;

  private Timer timer = new Timer();
  public PWMSparkFlex shiftLights = new PWMSparkFlex(9);

  public double matchTimer = DriverStation.getMatchTime();
  public double elapsedTime = Timer.getFPGATimestamp();

  boolean active = false;

  double switchActive = 0;

  public boolean isMatch = true;

  public boolean hasAlliance = false;

  public boolean isTeleop = DriverStation.isTeleop();

  public boolean isActive = true;

  public boolean wonAuto = false;

  public boolean lostAuto = true;

  public static boolean isBlueAlliance = true;

  /** Creates a new GameManager. */
  public GameManager() {
    // SmartDashboard.putData("Win Auto?", runOnce(() -> wonAuto()));
    // SmartDashboard.putData("Lost Auto?", runOnce(() -> lostAuto()));

    alwaysActiveShifts.add(ShiftList.Autonomous);
    alwaysActiveShifts.add(ShiftList.Transistion);
    LoseShifts.add(ShiftList.One);
    LoseShifts.add(ShiftList.Three);
    alwaysActiveShifts.add(ShiftList.EndGame);
    timer.reset();
    timer.start();
    winChooser = new SendableChooser<>();
winChooser.setDefaultOption("lost", runOnce(() -> lostAuto()));
winChooser.addOption("win", runOnce(() -> wonAuto()));

 SmartDashboard.putData("winChooser", winChooser);
  }

  @Override
  public void periodic() {
   winChooser.getSelected();
    if(!hasAlliance){
     Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() ) {
      
      if (alliance.get() == Alliance.Red) {
        isBlueAlliance = false;
        hasAlliance = true;
      }
      if (alliance.get() == Alliance.Blue) {
        isBlueAlliance = true;
        hasAlliance = true;
      }
    }
  }

    SmartDashboard.putBoolean("is blue?", isBlueAlliance);
    SmartDashboard.putBoolean("won Auto", wonAuto);
    SmartDashboard.putNumber("timer", matchTimer());

    isTeleop = DriverStation.isTeleop();
    matchTimer = DriverStation.getMatchTime();
    elapsedTime = Timer.getFPGATimestamp();

    determineShift();
    determineActiveHub();

    // SmartDashboard.putNumber("GameManager Timer", elapsedTime);

    // SmartDashboard.putString("current shift", currentShift.toString());

    // SmartDashboard.putNumber("match timer", matchTimer);
    // SmartDashboard.putBoolean("is teleop", isTeleop);
    // SmartDashboard.putBoolean("win auto?", wonAuto);

    // SmartDashboard.putBoolean("is active?", isActive);
    // SmartDashboard.putBoolean("is in a match?", isMatch);

    getMatchTime();

  }

  public void setLEDcolor( double setColor){
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
    // if (!wonAuto) {
    //   wonAuto = true;
    // } else {
    //   wonAuto = false;
    // }
    wonAuto = true;
  }

  public void switchActiveHubs() {
    if (!active) {
      active = true;
      switchActive = elapsedTime + 25;
    } else if (active) {
      active = false;
      switchActive = elapsedTime + 25;
    }

    // SmartDashboard.putBoolean("teleop active", active);
    // SmartDashboard.putNumber("time till swicth", switchActive);
  }

  public void lostAuto() {
    // if (!wonAuto) {
    //   lostAuto = true;
    // } else {
    //   lostAuto = false;
    // }
    wonAuto = false;
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
  public void resetTimer(){
    timer.reset();
  }
  public double matchTimer(){
    return timer.get();
  }
  public void determineActiveHub() {
      double elapsedTime = matchTimer();
      double activeColor = 0.75; // dark green
      double nonActiveColor = 0.61; //red
      double autoColor = 0.85; //dark blue
      double transitionColor = 0.57;//hot pink
      double warningColor = -.07;// gold strobe
      double endgameColor = 0.91;//violet
      double defaultColor = -0.85;//shot red
   
      //auto
      if(elapsedTime <= 17){
        shiftLights.set(autoColor);
     //   System.out.println("auto");
     
        // driverController.setRumble(RumbleType.kBothRumble, 0);
        // manipulatorController.setRumble(RumbleType.kBothRumble, 0);
      }
      //warning
       else if( elapsedTime > 17 && elapsedTime <=20){
        shiftLights.set(warningColor);
        // driverController.setRumble(RumbleType.kBothRumble, .2);
        // manipulatorController.setRumble(RumbleType.kBothRumble, .2);
     //   System.out.println("warning");
      }
      //transistion
      else if( elapsedTime > 20 && elapsedTime <=27){
        shiftLights.set(transitionColor);
        
        // driverController.setRumble(RumbleType.kBothRumble, 0);
        // manipulatorController.setRumble(RumbleType.kBothRumble, 0);
      //  System.out.println("transistion");
      }
      //warning
       else if( elapsedTime > 27 && elapsedTime <=30){
        shiftLights.set(warningColor);
        // driverController.setRumble(RumbleType.kBothRumble, .2);
        // manipulatorController.setRumble(RumbleType.kBothRumble, .2);
      //   System.out.println("warning");
      }
      //phase 1
      else if( elapsedTime > 30 && elapsedTime <=52){
        
        if(wonAuto){
          shiftLights.set(nonActiveColor);
        }
        else{
            shiftLights.set(activeColor);
        }
        
        // driverController.setRumble(RumbleType.kBothRumble, 0);
        // manipulatorController.setRumble(RumbleType.kBothRumble, 0);
       //  System.out.println("phase 1");
      }
      //warning
       else if( elapsedTime > 52 && elapsedTime <=55){
        shiftLights.set(warningColor);
        // driverController.setRumble(RumbleType.kBothRumble, .2);
        // manipulatorController.setRumble(RumbleType.kBothRumble, .2);
      //   System.out.println("warning");
        
      }
      //phase 2
      else if( elapsedTime > 55 && elapsedTime <=77){
         if(wonAuto){
          shiftLights.set(activeColor);
        }
        else{
            shiftLights.set(nonActiveColor);
        }
        
        driverController.setRumble(RumbleType.kBothRumble, 0);
        manipulatorController.setRumble(RumbleType.kBothRumble, 0);
     //   System.out.println("phase 2");
      }
      // warning
       else if( elapsedTime > 77 && elapsedTime <=80){
        shiftLights.set(warningColor);
        // driverController.setRumble(RumbleType.kBothRumble, .2);
        // manipulatorController.setRumble(RumbleType.kBothRumble, .2);
     //   System.out.println("warning");
      }
      //phase 3
      else if( elapsedTime > 80 && elapsedTime <=102){
         if(wonAuto){
          shiftLights.set(nonActiveColor);
        }
        else{
            shiftLights.set(activeColor);
        }
        
        driverController.setRumble(RumbleType.kBothRumble, 0);
        manipulatorController.setRumble(RumbleType.kBothRumble, 0);
      //  System.out.println("phase 3");
      }
      //warning
       else if( elapsedTime > 102 && elapsedTime <=105){
        shiftLights.set(warningColor);
        // driverController.setRumble(RumbleType.kBothRumble, .2);
        // manipulatorController.setRumble(RumbleType.kBothRumble, .2);
      //  System.out.println("warning");
      }
      // phase 4
      else if( elapsedTime > 105 && elapsedTime <=127){
         if(wonAuto){
          shiftLights.set(activeColor);
        }
        else{
            shiftLights.set(nonActiveColor);
        }
        
        driverController.setRumble(RumbleType.kBothRumble, 0);
        manipulatorController.setRumble(RumbleType.kBothRumble, 0);
      //  System.out.println("phase 4");
      }
        // warning
         else if( elapsedTime > 127 && elapsedTime <=130){
        shiftLights.set(warningColor);
        // driverController.setRumble(RumbleType.kBothRumble, .2);
        // manipulatorController.setRumble(RumbleType.kBothRumble, .2);
      //   System.out.println("warning");
      }
      else if( elapsedTime > 130 && elapsedTime <=160){
        shiftLights.set(endgameColor);
     //    System.out.println("endgame");
     
        driverController.setRumble(RumbleType.kBothRumble, 0);
        manipulatorController.setRumble(RumbleType.kBothRumble, 0);
      }
      else{
        shiftLights.set(defaultColor);
      //   System.out.println("default");
      
        driverController.setRumble(RumbleType.kBothRumble, 0);
        manipulatorController.setRumble(RumbleType.kBothRumble, 0);
      }
      
    




    
    // if (isMatch) {
    //   if (wonAuto) {
    //     if (LoseShifts.contains(currentShift)) {
    //       // not active
    //        shiftLights.set(0.61);
    //       isActive = false;
    //     } else {
    //       // active
    //        shiftLights.set(0.77);
    //       isActive = true;
    //     }
    //   } else {
    //     if (LoseShifts.contains(currentShift) || alwaysActiveShifts.contains(currentShift)) {
    //       // active
    //        shiftLights.set(0.77);
    //       isActive = true;
    //     } else {
    //       // not active
    //       shiftLights.set(0.61);
    //       isActive = false;
    //     }
    //   }
    // } else if (elapsedTime > switchActive) {
    //   switchActiveHubs();
    //   if (!active) {
    //     shiftLights.set(0.61);
    //   } else {
    //     shiftLights.set(0.77);
    //   }
    // }
  }
}
