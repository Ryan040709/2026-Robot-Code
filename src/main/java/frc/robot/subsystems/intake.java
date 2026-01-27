package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

public class intake {

    private TalonFX intakeMotor1 = new TalonFX(10);
    private TalonFX intakeMotor2 = new TalonFX(10);

    public void MoveintakeMotor1(double targetSpeed) {
        intakeMotor1.set(targetSpeed);
    }

    public void MoveintakeMotor2(double targetSpeed) {
        intakeMotor2.set(targetSpeed);
    }

}