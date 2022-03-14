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

import static org.rivierarobotics.subsystems.climb.ClimbConstants.MAX_RADS;

public class Climb extends SubsystemBase {
    private static Climb climbMotors;
    public static Climb getInstance() {
        if (climbMotors == null) {
            climbMotors = new Climb();
        }
        return climbMotors;
    }

    private final WPI_TalonFX climbMaster;
    private final WPI_TalonFX climbFollower;
    private final PositionStateSpaceModel climbStateSpace;
    private boolean play = true;
    private final SystemIdentification sysId = new SystemIdentification(0.0, 10, 0.02);
    //TODO: Remove this encoder
    private final DutyCycleEncoder encoder;

    private Climb() {
        //TODO: Move to motion magic. see SwerveModules if you need to see how that looks like.
        // keep in mind you will need to convert angle to ticks and ticks to angle.
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

        this.climbMaster = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE_A);
        this.climbFollower = new WPI_TalonFX(MotorIDs.CLIMB_ROTATE_B);
        climbFollower.follow(climbMaster);
        setCoast(false);
        climbMaster.setInverted(true);
        climbFollower.setInverted(true);
        StatusFrameDemolisher.demolishStatusFrames(climbMaster, false);
        StatusFrameDemolisher.demolishStatusFrames(climbFollower, true);

        this.encoder = new DutyCycleEncoder(MotorIDs.CLIMB_ENCODER);
        this.encoder.setDistancePerRotation(2 * Math.PI);
    }

    public void setCoast(boolean coast) {
        if (coast) {
            climbMaster.setNeutralMode(NeutralMode.Coast);
            climbFollower.setNeutralMode(NeutralMode.Coast);
        } else {
            climbMaster.setNeutralMode(NeutralMode.Brake);
            climbFollower.setNeutralMode(NeutralMode.Brake);
        }
    }

    //TODO: Make set position take an angle in radians, convert to ticks, and set on climbMaster. (gearing & ticks)
    public void setPosition(double radians) {
        climbStateSpace.setPosition(radians);
    }

    public void setPlay(boolean play) {
        this.play = play;
    }

    public boolean getPlay() {
        return play;
    }

    //TODO: Remove
    public void followStateSpace() {
        //var climbVoltage = climbStateSpace.getAppliedVoltage(getAngle());
        double angle = getAngle();
        if(MathUtil.isWithinTolerance(angle, climbStateSpace.getTargetPosition(), 0.1)) return;
        double v = Math.min(Math.abs((climbStateSpace.getTargetPosition() - angle) * (12 / 0.5)), 12);
        setVoltage(-Math.signum((climbStateSpace.getTargetPosition() - angle)) * v);
        //setVoltage(-climbVoltage);
    }

    public void setVoltage(double voltage) {
        if (Math.abs(getAngle()) > MAX_RADS && Math.signum(-voltage) == Math.signum(getAngle())) {
            climbMaster.setVoltage(0);
            return;
        }
        climbMaster.setVoltage(voltage);
    }

    //TODO: convert motor encoder to angle via gearing & encoder resolution
    public double getAngle() {
        return encoder.getDistance();
    }
}
