package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final SparkMax m_drivingSparkMax;
    private final SparkMax m_turningSparkMax;

    private final CANcoder m_turningCANCoder;
    
    // Absolute offset for the CANcoder
    private final double m_chassisAngularOffset;

    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    /**
     * Constructs a SwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. This configuration is specific to the REV
     * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */
    public SwerveModule(int drivingCANId, int turningCANId, int turningCANCoderId, double chassisAngularOffset) {
        m_drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
        m_turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
        m_turningCANCoder = new CANcoder(turningCANCoderId);
        m_chassisAngularOffset = chassisAngularOffset;

        configureDevices();
        resetEncoders();
    }

    private void configureDevices() {
        // Factory reset, so we get the SPARKS MAX to a known state before configuring
        // them. This is useful in case a SPARK MAX is swapped out.
        // In 2025, configuration is done via a config object and applied.

        SparkMaxConfig drivingConfig = new SparkMaxConfig();
        SparkMaxConfig turningConfig = new SparkMaxConfig();

    
        // Driving Motor Configuration
        drivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        
        drivingConfig.encoder
            .positionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR);

        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.DRIVING_P, ModuleConstants.DRIVING_I, ModuleConstants.DRIVING_D)
            .velocityFF(ModuleConstants.DRIVING_FF)
            .outputRange(ModuleConstants.DRIVING_MIN_OUTPUT, ModuleConstants.DRIVING_MAX_OUTPUT);


        // Turning Motor Configuration
        turningConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);

        turningConfig.encoder
            .positionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR);

        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ModuleConstants.TURNING_P, ModuleConstants.TURNING_I, ModuleConstants.TURNING_D)
            .outputRange(ModuleConstants.TURNING_MIN_OUTPUT, ModuleConstants.TURNING_MAX_OUTPUT)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(ModuleConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT, ModuleConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

        // Apply configurations
        m_drivingSparkMax.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turningSparkMax.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            m_drivingSparkMax.getEncoder().getVelocity(),
            new Rotation2d(m_turningSparkMax.getEncoder().getPosition())
        );
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            m_drivingSparkMax.getEncoder().getPosition(),
            new Rotation2d(m_turningSparkMax.getEncoder().getPosition())
        );
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState,
                new Rotation2d(m_turningSparkMax.getEncoder().getPosition()));

        // Command driving and turning SPARKS MAX towards their respective setpoints
        m_drivingSparkMax.getClosedLoopController().setReference(optimizedState.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        m_turningSparkMax.getClosedLoopController().setReference(optimizedState.angle.getRadians(), SparkMax.ControlType.kPosition);

        m_desiredState = desiredState;
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingSparkMax.getEncoder().setPosition(0);
        
        // Sync absolute encoder to relative encoder
        double absolutePosition = m_turningCANCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI; // Convert rotations to radians
        absolutePosition -= m_chassisAngularOffset; // Remove offset
        m_turningSparkMax.getEncoder().setPosition(absolutePosition);
    }
}
