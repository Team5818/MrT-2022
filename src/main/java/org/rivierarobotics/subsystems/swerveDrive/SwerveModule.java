package org.rivierarobotics.subsystems.swerveDrive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.util.StateSpace.PositionStateSpaceModel;
import org.rivierarobotics.util.StateSpace.SystemIdentification;
import org.rivierarobotics.util.StateSpace.VelocityStateSpaceModel;

public class SwerveModule {
    private static final double WHEEL_RADIUS = 0.0508;
    private final double zeroTicks;
    private static final int ENCODER_RESOLUTION = 4096;

    private final CANSparkMax driveMotor;
    private final VelocityStateSpaceModel driveController;
    private double currDriveVoltage = 0;
    private double currSteerVoltage = 0;
    private double targetVelocity = 0;
    private final SystemIdentification dmSID = new SystemIdentification(0.0, 7.0 / 2.5, 7 / (2.5 / 0.01));

    private final WPI_TalonSRX steeringMotor;
    private static final double STEER_MOTOR_TICK_TO_ANGLE = 2 * Math.PI / ENCODER_RESOLUTION;
    private final PositionStateSpaceModel steerController;
    private Rotation2d targetRotation = new Rotation2d(0);
    private Rotation2d targetRotationClamped = new Rotation2d(0);
    private final SystemIdentification tmSID = new SystemIdentification(0.0, 7.0 / 4.0, 7 / (4.0 / 0.01));

    /**
     * Constructs a SwerveModule.
     *
     * @param driveMotorChannel    ID for the drive motor.
     * @param steeringMotorChannel ID for the turning motor.
     * @param zeroTicks           ticks when angle = 0
     */
    public SwerveModule(int driveMotorChannel, int steeringMotorChannel, double zeroTicks, boolean driveInverted, boolean steeringInverted) {
        this.driveMotor = new CANSparkMax(driveMotorChannel, CANSparkMaxLowLevel.MotorType.kBrushless);
        this.steeringMotor = new WPI_TalonSRX(steeringMotorChannel);
        this.zeroTicks = zeroTicks;

        driveMotor.getEncoder().setPositionConversionFactor(2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION);
        driveMotor.getEncoder().setVelocityConversionFactor((2 * Math.PI * WHEEL_RADIUS / ENCODER_RESOLUTION) * 10);

        steeringMotor.setInverted(!steeringInverted);
        driveMotor.setInverted(driveInverted);


        this.driveController = new VelocityStateSpaceModel(
                dmSID, 0.1, 0.01,
                0.1, 0.00015, 12
        );

        this.steerController = new PositionStateSpaceModel(
                tmSID, 1, 1,
                0.01, 0.01, 0.1,
                0.08, 12
        );
    }

    private double clampAngle(double angle) {
        double low = -Math.PI;
        double high = Math.PI;
        angle = MathUtil.wrapToCircle(angle, 2 * Math.PI);
        if (angle > high) {
            angle -= 2 * Math.PI;
        }
        if (angle < low) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    public double getAbsoluteAngle() {
        return clampAngle(getAngle());
    }

    public double getAngle() {
        return (steeringMotor.getSensorCollection().getPulseWidthPosition() - zeroTicks) * STEER_MOTOR_TICK_TO_ANGLE;
    }

    public double getPosTicks() {
        return steeringMotor.getSensorCollection().getPulseWidthPosition();
    }

    public double getDriveVoltage() {
        return currDriveVoltage;
    }

    public double getSteerVoltage() {
        return currSteerVoltage;
    }

    public double getVelocity() {
        return driveMotor.getEncoder().getVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), new Rotation2d(clampAngle(getAngle())));
    }

    public void setDriveMotorVelocity(double metersPerSecond) {
        driveController.setVelocity(metersPerSecond);
    }

    public void setSteeringMotorAngle(double angleInRad) {
        steerController.setPosition(angleInRad);
    }

    public void setDriveMotorVoltage(double voltage) {
        currDriveVoltage = voltage;
        driveMotor.setVoltage(voltage);
    }

    public void setSteeringMotorVoltage(double voltage) {
        currSteerVoltage = voltage;
        steeringMotor.setVoltage(voltage);
    }

    public double getAngleDiff(double src, double target) {
        double diff = target - src;
        if (Math.abs(diff) <= Math.PI) {
            return diff;
        }

        if (diff > 0) {
            diff -= 2 * Math.PI;
        } else {
            diff += 2 * Math.PI;
        }

        return diff;
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        //Update State-Space Controllers
        double targetSpeed = state.speedMetersPerSecond;
        double targetRotation = state.angle.getRadians();
        double currAng = getAngle();
        double clampedAng = clampAngle(getAngle());


        double posTarget = targetRotation + Math.PI;
        double negTarget = targetRotation - Math.PI;
        double diff;

        if (Math.abs(getAngleDiff(clampedAng, posTarget)) <= Math.abs(getAngleDiff(clampedAng, negTarget))) {
            diff = getAngleDiff(clampedAng, posTarget);
        } else {
            diff = getAngleDiff(clampedAng, negTarget);
        }

        if (Math.abs(getAngleDiff(clampedAng, targetRotation)) <= Math.abs(diff)) {
            diff = getAngleDiff(clampedAng, targetRotation);
        }

        double targetAng = currAng + diff;

        if (MathUtil.isWithinTolerance(targetRotation, clampAngle(targetAng), 0.1)) {
            targetSpeed = state.speedMetersPerSecond;
        } else {
            targetSpeed = -state.speedMetersPerSecond;
        }

        this.targetRotation = new Rotation2d(targetAng);
        this.targetRotationClamped = new Rotation2d(clampAngle(targetAng));
        this.targetVelocity = targetSpeed;

        setDriveMotorVelocity(targetSpeed);
        setSteeringMotorAngle(targetAng);
    }

    public Rotation2d getTargetRotation() {
        return targetRotation;
    }

    public Rotation2d getTargetRotationClamped() {
        return targetRotationClamped;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getSteerMotorVel() {
        return steeringMotor.getSensorCollection().getPulseWidthVelocity() * 10 * STEER_MOTOR_TICK_TO_ANGLE;
    }

    public void periodic() {
        var driveVoltage = driveController.getAppliedVoltage(getVelocity());
        setDriveMotorVoltage(driveVoltage);

        var turnMotorVoltage = steerController.getAppliedVoltage(getAngle());
        setSteeringMotorVoltage(turnMotorVoltage);
    }
}