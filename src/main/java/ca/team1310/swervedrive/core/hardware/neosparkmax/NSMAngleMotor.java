package ca.team1310.swervedrive.core.hardware.neosparkmax;

import ca.team1310.swervedrive.core.AngleMotor;
import ca.team1310.swervedrive.core.config.MotorConfig;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class NSMAngleMotor extends NSMMotor implements AngleMotor {

    public NSMAngleMotor(int canId, MotorConfig cfg) {
        super(canId);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(cfg.inverted())
            .voltageCompensation(cfg.nominalVoltage())
            .smartCurrentLimit(cfg.currentLimitAmps())
            .closedLoopRampRate(cfg.rampRateSecondsZeroToFull())
            .idleMode(IdleMode.kBrake);

        // configure integrated encoder
        final double angleConversionFactor = 360 / cfg.gearRatio();

        config.encoder
            // report in degrees not rotations
            .positionConversionFactor(angleConversionFactor)
            // report in degrees per second not rotations per minute
            .velocityConversionFactor(angleConversionFactor / 60);

        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0,0,0)
            .velocityFF(cfg.ff(), ClosedLoopSlot.kSlot0)
            .iZone(cfg.izone(), ClosedLoopSlot.kSlot0)
            .outputRange(-180, 180, ClosedLoopSlot.kSlot0)
            .positionWrappingEnabled(true)
            .positionWrappingMinInput(-180)
            .positionWrappingMaxInput(180);

        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getPosition() {
        return (encoder.getPosition() + 360) % 360;
    }

    @Override
    public void setReferenceAngle(double degrees) {
        configureSparkMax(() -> pid.setReference(degrees, SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0));
    }

    @Override
    public void setEncoderPosition(double actualAngleDegrees) {
        if (encoder.getPosition() != actualAngleDegrees) {
            configureSparkMax(() -> encoder.setPosition(actualAngleDegrees));
        }
    }
}
