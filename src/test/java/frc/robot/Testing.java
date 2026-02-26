package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.TurretSubsystem;

import frc.robot.subsystems.CommandSwerveDrivetrain;

import edu.wpi.first.math.MathUtil;

class IntakeTest {

  //public final CommandSwerveDrivetrain drivetrain = SwerveConstants.createDrivetrain();

  

  // @Test // marks this method as a test
  // void test_FirstPose() {
  //   TurretSubsystem turret = new TurretSubsystem(() -> new Pose2d(1,1,0));

  //   assertEquals(turret.calculateAngleToHub(), 0);
  // }

  // @Test
  // void test_SecondPose() {
  //   TurretSubsystem turret = new TurretSubsystem(() -> new Pose2d(1,1,45));

  //   assertEquals(turret.calculateAngleToHub(), 0);
  // }

  @Test
  void distanceFromHub(){
    Pose2d robotPose2d = new Pose2d(2,0,Rotation2d.fromDegrees(90));
    Transform2d turretOffset = new Transform2d(1,0,Rotation2d.fromDegrees(0));

    Pose2d result = robotPose2d.transformBy(turretOffset);
    assertEquals(new Pose2d(2,1,Rotation2d.fromDegrees(90)),result);
  }

  // @Test 
  // void offsetAngleToHub(){
  //   double Hx= 4.62554;
  //   double velocityX = 1;
  //   double velocityY = 0;


  //       double hubOffsetX = Hx + (velocityX * 1);
  //       double hubOffsetY = 4.62534 + (velocityY * 1);
  //       double originalHubAngle;
  //       double turretTxOffset;

  //       double offsetInDegrees = Math
  //               .toDegrees(Math.atan((hubOffsetY - 1) / (hubOffsetX - 1)));

  //       double diffY = (4.62534 - 1);
  //       double diffX = (Hx - 1);
  //       originalHubAngle = Math.toDegrees(Math.atan2(diffY, diffX));

  //       turretTxOffset = originalHubAngle - offsetInDegrees;
  //       assertEquals(turretTxOffset, 6.9102854777);
  // }

  @Test
  void turretAngle() {

    double theta = 0;

        double diffX = (4.03606 - 1.7);
        double diffY = (4.62534 - 6.63);
        double turretHubAngle = Math.toDegrees(Math.atan2(diffY, diffX));
        double goldenAngle = MathUtil.clamp(MathUtil.inputModulus((turretHubAngle - theta), -30, 330), -20, 315);

        assertEquals(goldenAngle, -40.6341211707);
  }


}