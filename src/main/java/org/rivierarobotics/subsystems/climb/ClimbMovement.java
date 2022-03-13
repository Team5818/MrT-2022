package org.rivierarobotics.subsystems.climb;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.rivierarobotics.lib.MathUtil;
import org.rivierarobotics.subsystems.MotorIDs;
import org.rivierarobotics.util.StatusFrameDemolisher;
import org.rivierarobotics.util.statespace.PositionStateSpaceModel;
import org.rivierarobotics.util.statespace.SystemIdentification;

import static org.rivierarobotics.subsystems.MotorIDs.CLIMB_ENCODER;
import static org.rivierarobotics.subsystems.climb.ClimbValues.MAX_RADS;

public class ClimbMovement extends SubsystemBase {
    private static ClimbMovement climbMotors;
    public static ClimbMovement getInstance() {
        if (climbMotors == null) {
            climbMotors = new ClimbMovement();
        }
        return climbMotors;
    }

    private final WPI_TalonFX climbMotorA;
    private final WPI_TalonFX climbMotorB;
    private final PositionStateSpaceModel climbStateSpace;
    private boolean play = true;
    private final SystemIdentification sysId = new SystemIdentification(0.0, 10, 0.02);

    private ClimbMovement() {
        this.climbStateSpace = new PositionStateSpaceModel(
                sysId,
                0.01,
                0.01,
                0.01,
                0.01,
                0.01,
                6,
                12
        );

        this.climbMotorA = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE_A);
        this.climbMotorB = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE_B);
        climbMotorB.follow(climbMotorA);
        setCoast(false);
        climbMotorA.setInverted(true);
        climbMotorB.setInverted(true);
        StatusFrameDemolisher.demolishStatusFrames(climbMotorA, false);
        StatusFrameDemolisher.demolishStatusFrames(climbMotorB, true);
    }

    public void setCoast(boolean coast) {
        if (coast) {
            climbMotorA.setNeutralMode(NeutralMode.Coast);
            climbMotorB.setNeutralMode(NeutralMode.Coast);
        } else {
            climbMotorA.setNeutralMode(NeutralMode.Brake);
            climbMotorB.setNeutralMode(NeutralMode.Brake);
        }
    }

    public void setPosition(double radians) {
        climbStateSpace.setPosition(radians);
    }

    public void setPlay(boolean play) {
        this.play = play;
    }

    public boolean getPlay() {
        return play;
    }

    public void followStateSpace() {
        //var climbVoltage = climbStateSpace.getAppliedVoltage(getAngle());
        double angle = ClimbEncoder.getInstance().getAngle();
        if(MathUtil.isWithinTolerance(angle, climbStateSpace.getTargetPosition(), 0.1)) return;
        double v = Math.min(Math.abs((climbStateSpace.getTargetPosition() - angle) * (12 / 0.5)), 12);
        setVoltage(-Math.signum((climbStateSpace.getTargetPosition() - angle)) * v);
        //setVoltage(-climbVoltage);
    }

    public void setVoltage(double voltage) {
        if (Math.abs(ClimbEncoder.getInstance().getAngle()) > MAX_RADS && Math.signum(-voltage) == Math.signum(ClimbEncoder.getInstance().getAngle())) {
            climbMotorA.setVoltage(0);
            return;
        }
        climbMotorA.setVoltage(voltage);
    }

    public double getRawTicks() {
        return climbMotorA.getSelectedSensorPosition();
    }
}
