package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.utils.MathUtils;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex shooterMotor = new SparkFlex(ShooterConstants.shooterMotorID, SparkLowLevel.MotorType.kBrushless);

    private final SimpleMotorFeedforward shooterFF = new SimpleMotorFeedforward(
        ShooterConstants.shooterFFKs,
        ShooterConstants.shooterFFKv,
        ShooterConstants.shooterFFKa
    );

    private final PIDController shooterPID = new PIDController(
        ShooterConstants.shooterP,
        ShooterConstants.shooterI,
        ShooterConstants.shooterD
    );

    public ShooterSubsystem() {

    }


    public void setMotorRPM(double rpm) {
        shooterMotor.setVoltage(
            shooterPID.calculate(MathUtils.rpmToRadians(shooterMotor.getEncoder().getVelocity()), MathUtils.rpmToRadians(rpm)) +
                    shooterPID.calculate(MathUtils.rpmToRadians(rpm))
        );
    }

    public void runVolts(double volts) {
        shooterMotor.setVoltage(volts);
    }

    public void resetPID() {
        shooterPID.reset();
    }
}
