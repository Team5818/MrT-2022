/*
 * This file is part of Placeholder-2022, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.util.statespace;


import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import org.rivierarobotics.lib.MathUtil;

/**
 * State-Space Control System using the FRC Characterization method - kS kV and kA.
 */
public class PositionStateSpaceModel {
    private final LinearSystemLoop<N2, N1, N1> linearSystemLoop;
    private final SystemIdentification systemIdentification;
    private double targetPosition;
    private double ksTolerance = 0.0;
    private final double loopTime;

    /**
     * State-Space Position Control System using the FRC Characterization method - kV and kA.
     *
     * @param systemIdentification kS (static friction) kV (volts/units/s) kA (volts/units/s*s)
     * @param positionAccuracy how accurate we think our position model is (higher is more aggressive)
     * @param velocityAccuracy how accurate we think our velocity model is (higher is more aggressive)
     * @param positionErrorTolerance how tolerable we are to position error (lower is more aggressive)
     * @param velocityErrorTolerance how tolerable we are to velocity error (lower is more aggressive)
     * @param voltageControlEffort decrease this to penalize control effort (using more voltage)
     * @param maxVoltage the maximum voltage to apply to the motor (use this to limit speed)
     */
    public PositionStateSpaceModel(
        SystemIdentification systemIdentification, double positionAccuracy,
        double velocityAccuracy, double encoderAccuracy,
        double positionErrorTolerance, double velocityErrorTolerance,
        double voltageControlEffort, double maxVoltage
    ) {
        this(systemIdentification, positionAccuracy, velocityAccuracy,
            encoderAccuracy, positionErrorTolerance,
            velocityErrorTolerance, voltageControlEffort, maxVoltage,
            0.02);
    }

    /**
     * State-Space Position Control System using the FRC Characterization method - kV and kA.
     *
     * @param systemIdentification kS (static friction) kV (volts/units/s) kA (volts/units/s*s)
     * @param positionAccuracy how accurate we think our position model is (higher is more aggressive)
     * @param velocityAccuracy how accurate we think our velocity model is (higher is more aggressive)
     * @param positionErrorTolerance how tolerable we are to position error (lower is more aggressive)
     * @param velocityErrorTolerance how tolerable we are to velocity error (lower is more aggressive)
     * @param voltageControlEffort decrease this to penalize control effort (using more voltage)
     * @param maxVoltage the maximum voltage to apply to the motor (use this to limit speed)
     * @param loopTime if you want to run the system faster than the default loop time of 20ms, use this
     */
    public PositionStateSpaceModel(
        SystemIdentification systemIdentification, double positionAccuracy,
        double velocityAccuracy, double encoderAccuracy,
        double positionErrorTolerance, double velocityErrorTolerance,
        double voltageControlEffort, double maxVoltage, double loopTime
    ) {
        this.loopTime = loopTime;
        this.systemIdentification = systemIdentification;

        LinearSystem<N2, N1, N1> linearPositionSystem =
            LinearSystemId.identifyPositionSystem(systemIdentification.kV, systemIdentification.kA);

        KalmanFilter<N2, N1, N1> observer = new KalmanFilter<>(
            Nat.N2(), Nat.N1(),
            linearPositionSystem,
            VecBuilder.fill(positionAccuracy, velocityAccuracy),
            VecBuilder.fill(encoderAccuracy),
            loopTime
        );

        LinearQuadraticRegulator<N2, N1, N1> controller
            = new LinearQuadraticRegulator<>(linearPositionSystem,
            VecBuilder.fill(positionErrorTolerance, velocityErrorTolerance),
            VecBuilder.fill(voltageControlEffort),
            loopTime
        );

        this.linearSystemLoop = new LinearSystemLoop<>(
            linearPositionSystem,
            controller,
            observer,
            maxVoltage - systemIdentification.kS,
            loopTime
        );
    }

    public void setPosition(double units) {
        linearSystemLoop.setNextR(units, 0);
        this.targetPosition = units;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Checks if the encoder is within the specified tolerance from the target position.
     *
     * @param units current encoder value
     * @param tolerance amount of units away from the target position
     * @return true if within tolerance
     */
    public boolean isWithinTolerance(double units, double tolerance) {
        return MathUtil.isWithinTolerance(units, targetPosition, tolerance);
    }

    /**
     * Sets the tolerance of units away from 0 at which the ks term will be ignored.
     *
     * @param tolerance tolerance to set
     */
    public void setKsTolerance(double tolerance) {
        this.ksTolerance = tolerance;
    }

    public double getAppliedVoltage(double units) {
        linearSystemLoop.correct(VecBuilder.fill(units));
        linearSystemLoop.predict(loopTime);
        var target = linearSystemLoop.getU(0);
        return target + (MathUtil.isWithinTolerance(units, 0, ksTolerance) ? 0 : (Math.signum(target) * systemIdentification.kS));
    }
}
