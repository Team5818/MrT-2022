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

package org.rivierarobotics.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.climb.ClimbSetAngle;
import org.rivierarobotics.commands.climb.RunClimb;
import org.rivierarobotics.commands.climb.WaitPiston;
import org.rivierarobotics.commands.drive.SetDriveAngle;
import org.rivierarobotics.commands.drive.SetWheelbaseAngle;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class ButtonConfiguration {
    public void initTeleop() {
        new JoystickButton(ControlMap.CO_DRIVER_LEFT, 1)
                .whenPressed(new RunClimb(false));
        new JoystickButton(ControlMap.CO_DRIVER_LEFT, 2)
                .whenPressed(new RunClimb(true));
        new JoystickButton(ControlMap.CO_DRIVER_RIGHT, 1)
                .whenPressed(new SequentialCommandGroup(new InstantCommand(() -> Climb.getInstance().setCoast(false)), new InstantCommand(() -> Climb.getInstance().setOffset())));
        new JoystickButton(ControlMap.CO_DRIVER_RIGHT, 2)
                .whenPressed(new InstantCommand(() -> Climb.getInstance().setCoast(false)));
        new JoystickButton(ControlMap.DRIVER_LEFT, 2)
                .whileHeld(new SetDriveAngle(-90, 0.1));
        new JoystickButton(ControlMap.DRIVER_RIGHT, 1)
                .whenPressed(new SetWheelbaseAngle(90).withTimeout(4));
        new JoystickButton(ControlMap.DRIVER_RIGHT, 2)
                .whenPressed(new SetWheelbaseAngle(180).withTimeout(4));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 1)
                .whenPressed(new ClimbSetAngle(0));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 2).whenPressed(new SetWheelbaseAngle(90).withTimeout(2));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 3).whenPressed(new SetWheelbaseAngle(0).withTimeout(2));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 4).whenPressed(new SetWheelbaseAngle(-90).withTimeout(2));
    }
}
