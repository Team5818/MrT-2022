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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.auto.SimpleAuto;
import org.rivierarobotics.commands.auto.TestPathGeneration;
import org.rivierarobotics.commands.climb.ClimbSetAngle;
import org.rivierarobotics.commands.climb.RunClimb;
import org.rivierarobotics.commands.climb.WaitPiston;
import org.rivierarobotics.commands.collect.CollectControl;
import org.rivierarobotics.commands.collect.CollectToggle;
import org.rivierarobotics.commands.drive.SetCameraCentric;
import org.rivierarobotics.commands.drive.SetDriveAngle;
import org.rivierarobotics.commands.drive.SetWheelbaseAngle;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

public class ButtonConfiguration {
    public void initTeleop() {
        //DRIVER JOYSTICK BUTTONS
        new JoystickButton(ControlMap.CO_DRIVER_RIGHT, 1)
                .toggleWhenPressed(new CollectToggle(true, true, true));

        new JoystickButton(ControlMap.CO_DRIVER_RIGHT, 2)
                .toggleWhenPressed(new CollectToggle(false, true, true));

        new JoystickButton(ControlMap.DRIVER_LEFT, 2)
                .whileHeld(new TestPathGeneration(1,1));
        new JoystickButton(ControlMap.DRIVER_LEFT, 1).
                whileHeld(new SimpleAuto());

        //CO-DRIVER JOYSTICK BUTTONS


        //DRIVER BUTTONS
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 1).whenPressed(new ClimbSetAngle(0));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 2).whenPressed(new SetWheelbaseAngle(90).withTimeout(2));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 3).whenPressed(new SetWheelbaseAngle(0).withTimeout(2));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 4).whenPressed(new SetWheelbaseAngle(-90).withTimeout(2));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 7).whenPressed(new InstantCommand(() -> DriveTrain.getInstance().setTargetRotationAngle(90)));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 8).whenPressed(new InstantCommand(() -> DriveTrain.getInstance().setTargetRotationAngle(0)));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 9).whenPressed(new InstantCommand(() -> {
            for (var sm : DriveTrain.getInstance().getSwerveModules()) {
                sm.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
            }
        }));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13).whenPressed(new SetCameraCentric());
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 10).whenPressed(new InstantCommand(() -> {
            for (var sm : DriveTrain.getInstance().getSwerveModules()) {
                sm.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI / 2)));
            }
        }));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 11).whenPressed(new InstantCommand(() -> {
            for (var sm : DriveTrain.getInstance().getSwerveModules()) {
                sm.setDesiredState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 2)));
            }
        }));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 6).whenPressed(new WaitPiston(Climb.Position.HIGH, 4, 8, false));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 7).whenPressed(new ClimbSetAngle(1));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 15).whenHeld(new SetCameraCentric());

        //CO-DRIVER BUTTONS
    }
}
