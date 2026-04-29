package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    /* log and replay timestamp and joystick data */
    // private final HootAutoReplay timeAndJoystickReplay = new HootAutoReplay()
    //     .withTimestampReplay()
    //     .withJoystickReplay();

    public Robot() {
        robotContainer = new RobotContainer();
      //  SmartDashboard.putData("robot", this.);
    }

    @Override
    public void robotPeriodic() {
       // timeAndJoystickReplay.update();
        Threads.setCurrentThreadPriority(true, 99);
        CommandScheduler.getInstance().run(); 
        Threads.setCurrentThreadPriority(false, 10);

    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(autonomousCommand);
        }
         robotContainer.teleopInit();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
