package org.rivierarobotics.util.StateSpace;


import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import org.rivierarobotics.lib.MathUtil;

/**
 * State-Space Control System using the FRC Characterization method - kS kV and kA.
 */
public class VelocityStateSpaceModel {
    private final LinearSystemLoop<N1, N1, N1> linearSystemLoop;
    private final SystemIdentification systemIdentification;
    private double targetVelocity;
    private final double loopTime;

    /**
     * State-Space Position Control System using the FRC Characterization method - kV and kA.
     *
     * @param systemIdentification   kS (static friction) kV (volts/units/s) kA (volts/units/s*s)
     * @param velocityAccuracy       how accurate we think our velocity model is (higher is more aggressive)
     * @param velocityErrorTolerance how tolerable we are to velocity error (lower is more aggressive)
     * @param voltageControlEffort   decrease this to penalize control effort (using more voltage)
     * @param maxVoltage             the maximum voltage to apply to the motor (use this to limit speed)
     */
    public VelocityStateSpaceModel(SystemIdentification systemIdentification,
                                   double velocityAccuracy, double encoderAccuracy,
                                   double velocityErrorTolerance, double voltageControlEffort,
                                   double maxVoltage) {
        this(systemIdentification, velocityAccuracy, encoderAccuracy,
                velocityErrorTolerance, voltageControlEffort, maxVoltage, 0.02);
    }

    /**
     * State-Space Position Control System using the FRC Characterization method - kV and kA.
     *
     * @param systemIdentification   kS (static friction) kV (volts/units/s) kA (volts/units/s*s)
     * @param velocityAccuracy       how accurate we think our velocity model is (higher is more aggressive)
     * @param velocityErrorTolerance how tolerable we are to velocity error (lower is more aggressive)
     * @param voltageControlEffort   decrease this to penalize control effort (using more voltage)
     * @param maxVoltage             the maximum voltage to apply to the motor (use this to limit speed)
     * @param loopTime               if you want to run the system faster than the default loop time of 20ms, use this
     */
    public VelocityStateSpaceModel(SystemIdentification systemIdentification,
                                   double velocityAccuracy, double encoderAccuracy,
                                   double velocityErrorTolerance, double voltageControlEffort,
                                   double maxVoltage, double loopTime) {
        this.loopTime = loopTime;
        this.systemIdentification = systemIdentification;

        LinearSystem<N1, N1, N1> linearPositionSystem =
                LinearSystemId.identifyVelocitySystem(systemIdentification.kV, systemIdentification.kA);

        KalmanFilter<N1, N1, N1> observer = new KalmanFilter<>(
                Nat.N1(), Nat.N1(),
                linearPositionSystem,
                VecBuilder.fill(velocityAccuracy),
                VecBuilder.fill(encoderAccuracy),
                loopTime
        );

        LinearQuadraticRegulator<N1, N1, N1> controller
                = new LinearQuadraticRegulator<>(linearPositionSystem,
                VecBuilder.fill(velocityErrorTolerance),
                VecBuilder.fill(voltageControlEffort),
                loopTime
        );

        this.linearSystemLoop = new LinearSystemLoop<>(
                linearPositionSystem,
                controller,
                observer,
                maxVoltage,
                loopTime
        );
    }

    public void setVelocity(double unitsPerS) {
        linearSystemLoop.setNextR(unitsPerS);
        targetVelocity = unitsPerS;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isWithinTolerance(double unitsPerS, double tolerance) {
        return MathUtil.isWithinTolerance(unitsPerS, targetVelocity, tolerance);
    }

    public double getAppliedVoltage(double unitsPerS) {
        linearSystemLoop.correct(VecBuilder.fill(unitsPerS));
        linearSystemLoop.predict(loopTime);
        return linearSystemLoop.getU(0) + (MathUtil.isWithinTolerance(unitsPerS, 0, 0.01) ? 0 : systemIdentification.kS);
    }
}
