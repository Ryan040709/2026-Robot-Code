import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.subsystems.TurretSubsystem;

import frc.robot.subsystems.CommandSwerveDrivetrain;

class IntakeTest {

  public final CommandSwerveDrivetrain drivetrain = SwerveConstants.createDrivetrain();

  

  @Test // marks this method as a test
  void test_FirstPose() {
    TurretSubsystem turret = new TurretSubsystem(() -> new Pose2d(1,1,0));

    assertEquals(turret.calculateAngleToHub(), 0);
  }
  @test
  void test_SecondPose() {
    TurretSubsystem turret = new TurretSubsystem(() -> new Pose2d(1,1,45));

    assertEquals(turret.calculateAngleToHub(), 0);
  }
}