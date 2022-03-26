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

package org.rivierarobotics.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.advanced.climb.ClimbInterruptToggle;
import org.rivierarobotics.commands.advanced.climb.RunClimb;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.auto.CrazyWildCollectionBouncyHousePath;
import org.rivierarobotics.commands.auto.DrivePathPlannerPath;
import org.rivierarobotics.commands.basic.climb.ClimbSetVoltage;
import org.rivierarobotics.commands.basic.collect.ToggleIntakeState;
import org.rivierarobotics.commands.basic.drive.*;
import org.rivierarobotics.commands.basic.shoot.SetFlywheelSpeed;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;

public class ButtonConfiguration {
    public void initTeleop() {
        //Driver Left
        new JoystickButton(ControlMap.DRIVER_LEFT, 1)
                .toggleWhenPressed(new SetWheelbaseAngle(0));
        new JoystickButton(ControlMap.DRIVER_LEFT, 2)
                .toggleWhenPressed(new SetWheelbaseAngle(90));
        //Driver Right
        new JoystickButton(ControlMap.DRIVER_RIGHT, 1)
                .whenPressed(new ToggleIntakeState());
        new JoystickButton(ControlMap.DRIVER_RIGHT, 2)
                .toggleWhenPressed(new CollectBalls());

        //Driver Buttons
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 2)
                .whenPressed(new AutoAimShoot());
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 7)
                .whenPressed(new ClimbInterruptToggle());
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 8)
                .whenPressed(new ClimbSetVoltage(false, 0));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 9)
                .whenPressed(new RunClimb(false));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 11)
                .whenPressed(new RunClimb(true));

        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13)
                .whileHeld(new SetDriverAssist(true));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13)
                .whenReleased(new SetDriverAssist(false));

        new JoystickButton(ControlMap.DRIVER_BUTTONS, 15)
                .whenPressed(new SetCameraCentric());

        //CO-DRIVER

        //CO-DRIVER JOYSTICK
        new JoystickButton(ControlMap.CO_DRIVER_LEFT, 1)
                .whileHeld(new DrivePathPlannerPath("forward90", 2, 0.5));

        //CO-DRIVER BUTTONS

        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1)
                .whenPressed(new SetDriveAngle(-20));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2)
                .whenPressed(new SetDriveAngle(-180));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2)
                .whenPressed(new SetDriveAngle(180));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 3)
                .whileHeld(new SetFlywheelSpeed(0));
        //TODO if these are going to be here, make actual command classes for them
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 4)
                .whenPressed(new InstantCommand(() -> FloppaFlywheels.getInstance().setTargetVelocity(FloppaFlywheels.getInstance().getTargetVelocity() - 200)));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 5)
                .whenPressed(new InstantCommand(() -> FloppaFlywheels.getInstance().setTargetVelocity(FloppaFlywheels.getInstance().getTargetVelocity() + 200)));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 6)
                .whenPressed(new CrazyWildCollectionBouncyHousePath());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 7)
                .whileHeld(new AngulationToTargetBasedOffOfPose());
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8)
//                .whileHeld(new SequentialCommandGroup(new SetDriveAngle(77)));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8)
                .whenPressed(new InstantCommand(() -> {
                    DriveTrain.getInstance().getPoseEstimator().resetPose(Limelight.getInstance().getLLAbsPose());
                }));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 9)
//                .whenPressed(new InstantCommand(() -> {
//                    DriveTrain.getInstance().setMinTurnSpeed(DriveTrain.getInstance().getMinTurnSpeed() - 0.001);
//                }));
    }
}
