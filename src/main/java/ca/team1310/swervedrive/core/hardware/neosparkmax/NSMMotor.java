package ca.team1310.swervedrive.core.hardware.neosparkmax;

import com.revrobotics.*;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.function.Supplier;

class NSMMotor {
    /**
     * The maximum amount of times the swerve motor will attempt to configure a motor if failures
     * occur.
     */
    private final int                           maximumRetries = 5;
    protected final SparkMax                    motor;
    protected final RelativeEncoder             encoder;
    protected final SparkClosedLoopController   pid;

    NSMMotor(int canBusId) {
        // instantiate & configure motor
        this.motor   = new SparkMax(canBusId, SparkLowLevel.MotorType.kBrushless);
        this.encoder = this.motor.getEncoder();
        this.pid     = motor.getClosedLoopController();
        
        configureCANStatusFrames(10, 20, 20, 500, 500);
    }

    /**
     * Run the configuration until it succeeds or times out.
     *
     * @param config Lambda supplier returning the error state.
     */
    protected final void configureSparkMax(Supplier<REVLibError> config) {
        for (int i = 0; i < maximumRetries; i++) {
            if (config.get() == REVLibError.kOk) {
                return;
            }
        }
        DriverStation.reportWarning("Failure configuring motor " + motor.getDeviceId(), true);
    }

    /**
     * Set the CAN status frames.
     *
     * @param CANStatus0 Applied Output, Faults, Sticky Faults, Is Follower
     * @param CANStatus1 Motor Velocity, Motor Temperature, Motor Voltage, Motor Current
     * @param CANStatus2 Motor Position
     * @param CANStatus3 Analog Sensor Voltage, Analog Sensor Velocity, Analog Sensor Position
     * @param CANStatus4 Alternate Encoder Velocity, Alternate Encoder Position
     */
    private void configureCANStatusFrames(int CANStatus0, int CANStatus1, int CANStatus2, int CANStatus3, int CANStatus4) {
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.signals
            .appliedOutputPeriodMs(CANStatus0)
            .faultsPeriodMs(CANStatus0)
            .motorTemperaturePeriodMs(CANStatus1)
            .primaryEncoderPositionPeriodMs(CANStatus2)
            .analogPositionPeriodMs(CANStatus3)
            .analogVelocityPeriodMs(CANStatus3)
            .analogVoltagePeriodMs(CANStatus3)
            .absoluteEncoderPositionPeriodMs(CANStatus4)
            .absoluteEncoderVelocityPeriodMs(CANStatus4);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // TODO: Configure Status Frame 5 and 6 if necessary
        // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
    }
}
