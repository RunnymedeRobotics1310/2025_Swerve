package ca.team1310.swervedrive.core.hardware.neosparkmax;

import ca.team1310.swervedrive.core.DriveMotor;
import ca.team1310.swervedrive.core.config.MotorConfig;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class NSMDriveMotor extends NSMMotor implements DriveMotor {
    public NSMDriveMotor(int canId, MotorConfig cfg, double wheelRadiusMetres) {
        super(canId);

        SparkMaxConfig config = new SparkMaxConfig();

        // instantiate & configure motor
        config
            .inverted(cfg.inverted())
            .voltageCompensation(cfg.nominalVoltage())
            .smartCurrentLimit(cfg.currentLimitAmps())
            .closedLoopRampRate(cfg.rampRateSecondsZeroToFull())
            .idleMode(IdleMode.kBrake);

        // configure integrated encoder
        final double positionConversionFactor = (2 * Math.PI * wheelRadiusMetres) / cfg.gearRatio();

        config.encoder
            // report in metres not rotations
            .positionConversionFactor(positionConversionFactor)
            // report in metres per second not rotations per minute
            .velocityConversionFactor(positionConversionFactor / 60);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0,0,0)
            .velocityFF(cfg.ff(), ClosedLoopSlot.kSlot0)
            .iZone(cfg.izone(), ClosedLoopSlot.kSlot0)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot0)
            .positionWrappingEnabled(true);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getDistance() {
        return encoder.getPosition();
    }

    @Override
    public void setReferenceVelocity(double targetVelocityMPS) {
        configureSparkMax(() -> pid.setReference(targetVelocityMPS, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0));
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity();
    }
}
