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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.basic.climb.ClimbSetPosition;
import org.rivierarobotics.commands.basic.climb.ClimbSetVoltage;
import org.rivierarobotics.commands.advanced.climb.RunClimb;
import org.rivierarobotics.commands.advanced.collect.CollectToggle;
import org.rivierarobotics.commands.basic.climb.IdleMode;
import org.rivierarobotics.commands.basic.collect.IntakeDeployToggle;
import org.rivierarobotics.commands.advanced.drive.PathGeneration;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.advanced.shoot.EjectOne;
import org.rivierarobotics.commands.advanced.shoot.Shoot;
import org.rivierarobotics.commands.basic.climb.ClimbSetAngle;
import org.rivierarobotics.commands.advanced.climb.ClimbInterruptToggle;
import org.rivierarobotics.commands.basic.drive.SetCameraCentric;
import org.rivierarobotics.commands.basic.drive.SetDriverAssist;
import org.rivierarobotics.commands.basic.shoot.SetFloppaPosition;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.shoot.Floppas;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.Gyro;

import static org.rivierarobotics.subsystems.climb.ClimbPositions.LOW;
import static org.rivierarobotics.subsystems.climb.ClimbPositions.MID;

public class ButtonConfiguration {
    public void initTeleop() {
        //Driver Left
        new JoystickButton(ControlMap.DRIVER_LEFT, 1)
                .toggleWhenPressed(new IdleMode(true));
        new JoystickButton(ControlMap.DRIVER_LEFT,2).whenPressed(new InstantCommand(() -> Intake.getInstance().setVoltages(-8, -12)));

        //Driver Right
        new JoystickButton(ControlMap.DRIVER_RIGHT, 1)
                .toggleWhenPressed(new ClimbSetAngle(2));
        new JoystickButton(ControlMap.DRIVER_RIGHT, 2)
                .toggleWhenPressed(new ClimbSetAngle(0));

        //Driver Buttons
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 1)
//                .whenPressed(new TrackBall());
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 2)
                .whenPressed(new AutoAimShoot());
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 3)
//                .whenPressed(new WaitCommand(1));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 4)
//                .whenPressed(new DriveToClosest());
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 5)
//                .whenPressed(new WaitCommand(1));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 6)
//                .whenPressed(new WaitCommand(1));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 7)
                .whenPressed(new ClimbInterruptToggle());
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 8)
//                .whenPressed(new InstantCommand(() -> {
//                    try {
//                        CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(Climb.getInstance()));
//                    } catch (Exception e) {
//                    }
//                    Climb.getInstance().setVoltage(0);
//                }));
        //
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 8)
                .whenPressed(new ClimbSetVoltage(false, 0));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 9)
                .whenPressed(new RunClimb(false));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 10)
//                .whenPressed(new WaitCommand(1));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 11)
                .whenPressed(new RunClimb(true));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 12)
//                .whenPressed(new WaitCommand(1));

        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13).whileHeld(new SetDriverAssist(true));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13).whenReleased(new SetDriverAssist(false));

        new JoystickButton(ControlMap.DRIVER_BUTTONS, 15).
                whenPressed(new SetCameraCentric());

        //CO-DRIVER

        //CO-DRIVER JOYSTICK
//        new JoystickButton(ControlMap.CO_DRIVER_LEFT,1).whenPressed()

        //CO-DRIVER BUTTONS
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).whileHeld(new AutoAimShoot());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2).whileHeld(new Shoot(Floppas.ShooterLocations.FENDER));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 3).whileHeld(new Shoot(Floppas.ShooterLocations.LOW_GOAL));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 4).whenPressed(new EjectOne());


        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 5).whileHeld(new SetFloppaPosition(-6.1));
        //new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 6).whenPressed(new SetFloppaPosition(-90));

//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).whileHeld();
        //new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 5).whileHeld(new TrackGoal());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 6).whenPressed(new InstantCommand(() -> {
            DriveTrain.getInstance().getPoseEstimator().resetPose(new Pose2d(Limelight.getInstance().getLLAbsPose(), Gyro.getInstance().getRotation2d()));
        }));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 7).whenPressed(new ClimbSetAngle(0));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8).whileHeld(new SetDriveAngle(Limelight.getInstance().getShootingAssistAngle()));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8).whileHeld(new PathGeneration(0,-2));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 9).whenPressed(new InstantCommand(() -> {
            DriveTrain.getInstance().setTargetRotationAngle(0);
            Gyro.getInstance().resetGyro();
        }));

        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 10).whenPressed(new InstantCommand(() -> {
            Floppas.getInstance().setTargetV(Floppas.getInstance().getTargetV() + 5);
        }));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 11).whenPressed(new InstantCommand(() -> {
            Floppas.getInstance().setTargetV(Floppas.getInstance().getTargetV() - 5);
        }));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 12).whenPressed(new Shoot(true));
    }
}
