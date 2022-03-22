/*
 * This file is part of MrT-2022, licensed under the GNU General Public License (GPLv3).
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

package org.rivierarobotics.commands.advanced.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbClaws;
import org.rivierarobotics.subsystems.climb.ClimbPositions;

public class WaitPiston extends CommandBase {
    private final Climb climb;
    private final ClimbClaws climbClaws;
    private final ClimbPositions climbModule;
    private double switchTime;
    private double retryTimeout;
    private double endTime;
    private double timeout;
    private boolean retryMode;
    private final boolean reversed;


    public WaitPiston(ClimbPositions climbModule, double endTime, double timeout, boolean reversed) {
        this.climb = Climb.getInstance();
        this.climbClaws = ClimbClaws.getInstance();
        this.climbModule = climbModule;
        this.endTime = endTime;
        this.timeout = timeout;
        this.reversed = reversed;
        this.addRequirements(this.climb, this.climbClaws);
    }

    @Override
    public void initialize() {
        this.switchTime = Timer.getFPGATimestamp();
        this.retryTimeout = this.switchTime;
        this.retryMode = false;
    }

    @Override
    public void execute() {
        if (retryMode) {
            if (climbClaws.isSwitchSet(climbModule)) {
                climbClaws.setPiston(climbModule, true);
                this.retryMode = false;
                this.retryTimeout = Timer.getFPGATimestamp();
                this.switchTime = Timer.getFPGATimestamp();
            }
            if (climb.getPlay()) {
                if (reversed) {
                    climb.setVoltage(4.5);
                } else {
                    climb.setVoltage(-4.5);
                }
            } else {
                climb.setVoltage(0);
            }
        } else {
            climb.setVoltage(0);
            if (Timer.getFPGATimestamp() <= retryTimeout + timeout) {
                if (!climbClaws.isSwitchSet(climbModule)) {
                    this.switchTime = Timer.getFPGATimestamp();
                }
                Logging.robotShuffleboard.getTab("Climb").setEntry("Switch time", switchTime);
            } else {
                climbClaws.setPiston(climbModule, false);
                this.retryMode = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return climbClaws.isPistonSet(climbModule) && Timer.getFPGATimestamp() >= switchTime + endTime;
    }
}
