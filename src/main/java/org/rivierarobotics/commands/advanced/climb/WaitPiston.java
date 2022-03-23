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
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbClaws;
import org.rivierarobotics.subsystems.climb.ClimbPositions;

public class WaitPiston extends CommandBase {
    private final Climb climb;
    private final ClimbClaws climbClaws;
    private final ClimbPositions climbModule;
    private final double endTime;
    private final double timeout;
    private double switchTime;
    private double retryTimeout;
    private boolean mode;
    private final double voltage;

    public WaitPiston(ClimbPositions climbModule, double endTime, double timeout, boolean reversed) {
        this.climb = Climb.getInstance();
        this.climbClaws = ClimbClaws.getInstance();
        this.climbModule = climbModule;
        this.endTime = endTime;
        this.timeout = timeout;
        this.voltage = reversed ? 4.5 : -4.5;
        this.addRequirements(this.climb, this.climbClaws);
    }

    @Override
    public void initialize() {
        this.switchTime = Timer.getFPGATimestamp();
        this.retryTimeout = this.switchTime +timeout;
        this.mode = true;
    }

    @Override
    public void execute() {
        if (mode) {
            climb.setVoltage(0);
            if (Timer.getFPGATimestamp() <= retryTimeout) {
                if (!climbClaws.isSwitchSet(climbModule)) {
                    this.switchTime = Timer.getFPGATimestamp() + endTime;
                }
            } else {
                climbClaws.setPiston(climbModule, false);
                this.mode = false;
            }
        } else {
            if (climbClaws.isSwitchSet(climbModule)) {
                climbClaws.setPiston(climbModule, true);
                this.retryTimeout = Timer.getFPGATimestamp() + timeout;
                this.switchTime = Timer.getFPGATimestamp() + endTime;
                this.mode = true;
            }
            climb.setVoltage(climb.getPlay() ? voltage : 0);
        }
    }

    @Override
    public boolean isFinished() {
        return climbClaws.isPistonSet(climbModule) && Timer.getFPGATimestamp() >= switchTime;
    }
}
