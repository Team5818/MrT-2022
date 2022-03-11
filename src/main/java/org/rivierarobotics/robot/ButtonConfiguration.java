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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.auto.MLAuto;
import org.rivierarobotics.commands.auto.PathGeneration;
import org.rivierarobotics.commands.auto.SimpleAuto;
import org.rivierarobotics.commands.climb.*;
import org.rivierarobotics.commands.collect.*;
import org.rivierarobotics.commands.drive.SetCameraCentric;
import org.rivierarobotics.commands.drive.SetDriverAssist;
import org.rivierarobotics.commands.drive.SetWheelbaseAngle;
import org.rivierarobotics.commands.drive.SwerveControl;
import org.rivierarobotics.commands.shoot.*;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.intake.Intake;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Floppas;
import org.rivierarobotics.util.Gyro;

public class ButtonConfiguration {
    public void initTeleop() {
//        //DRIVER JOYSTICK BUTTONS
//        new JoystickButton(ControlMap.DRIVER_RIGHT, 1)
//                .toggleWhenPressed(new CollectToggle(true, true, true));
//
//        new JoystickButton(ControlMap.DRIVER_RIGHT, 2)
//                .toggleWhenPressed(new CollectToggle(false, true, true));
//
//        //CO-DRIVER JOYSTICK BUTTONS
//
//        new JoystickButton(ControlMap.CO_DRIVER_LEFT, 1)
//                .whenPressed(new RunClimb(false));
//
//        new JoystickButton(ControlMap.CO_DRIVER_LEFT, 2)
//                .whenPressed(new RunClimb(true));
//
//
//        //DRIVER BUTTONS
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 1).toggleWhenPressed(new IntakeDeployToggle());
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 2).whenPressed(new SetWheelbaseAngle(90).withTimeout(2));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 3).whenPressed(new SetWheelbaseAngle(0).withTimeout(2));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 4).whenPressed(new SetWheelbaseAngle(-90).withTimeout(2));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 7).whenPressed(new InstantCommand(() -> DriveTrain.getInstance().setTargetRotationAngle(90)));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 8).whenPressed(new InstantCommand(() -> DriveTrain.getInstance().setTargetRotationAngle(0)));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 9).whenPressed(new InstantCommand(() -> {
//            for (var sm : DriveTrain.getInstance().getSwerveModules()) {
//                sm.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
//            }
//        }));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13).whenPressed(new SetCameraCentric());
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 10).whenPressed(new InstantCommand(() -> {
//            for (var sm : DriveTrain.getInstance().getSwerveModules()) {
//                sm.setDesiredState(new SwerveModuleState(0, new Rotation2d(Math.PI / 2)));
//            }
//        }));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 11).whenPressed(new InstantCommand(() -> {
//            for (var sm : DriveTrain.getInstance().getSwerveModules()) {
//                sm.setDesiredState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 2)));
//            }
//        }));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 6).whenPressed(new WaitPiston(Climb.Position.HIGH, 4, 8, false));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 15).whenHeld(new SetCameraCentric());
//
//        //CO-DRIVER BUTTONS
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).whenPressed(new ClimbSetAngle(0));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2).whenPressed(new Shoot(Floppas.ShooterLocations.FENDER));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 3).whenPressed(new Shoot(Floppas.ShooterLocations.LOW_GOAL));
//
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 4).whenPressed(new Shoot(Floppas.ShooterLocations.LAUNCHPAD_A));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 5).whenPressed(new Shoot(Floppas.ShooterLocations.LAUNCHPAD_B));
//
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 6).whenPressed(new Shoot());
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 7).whenPressed(new InstantCommand(() -> {Floppas.getInstance().setTargetV(Floppas.getInstance().getTargetV() + 5);}));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8).whenPressed(new InstantCommand(() -> {Floppas.getInstance().setTargetV(Floppas.getInstance().getTargetV() - 5);}));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 9).whenPressed(new AutoAimShoot());
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 10).whileHeld(new DriveToClosest());
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 11).whileHeld(new PathGeneration(1,-1));
//
//
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 12).toggleWhenPressed(new IntakeDeployToggle());
//
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 13).whileHeld(new TrackGoal());
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 15).whenPressed(new InstantCommand(() -> Climb.getInstance().setPlay(true)));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 15).whenReleased(new InstantCommand(() -> Climb.getInstance().setPlay(false)));

        //Competition Button Configs

//        //Driver Joystick Buttons
//        new JoystickButton(ControlMap.DRIVER_LEFT, 1).toggleWhenPressed(new CollectToggle(true, false, false));
//
//        new JoystickButton(ControlMap.DRIVER_RIGHT, 1).toggleWhenPressed(new IntakeDeployToggle());
//        new JoystickButton(ControlMap.DRIVER_RIGHT, 2).toggleWhenPressed(new CollectToggle(false, true, true));
//
//        //Driver Buttons
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 1).toggleWhenPressed(new TrackBall());
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 2).whenPressed(new AutoAimShoot());
////        new JoystickButton(ControlMap.DRIVER_BUTTONS, 3).whenPressed(new SetWheelbaseAngle(0).withTimeout(2));
////        new JoystickButton(ControlMap.DRIVER_BUTTONS, 4).whenPressed(new SetWheelbaseAngle(-90).withTimeout(2));
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 5).whenPressed(new InstantCommand(() -> DriveTrain.getInstance().setTargetRotationAngle(90)));
////        new JoystickButton(ControlMap.DRIVER_BUTTONS, 8).whenPressed(new InstantCommand(() -> DriveTrain.

        // Player Buttons

        //Driver Left
        new JoystickButton(ControlMap.DRIVER_LEFT, 1)
                .toggleWhenPressed(new CollectToggle(true, true, true));
//        new JoystickButton(ControlMap.DRIVER_LEFT,2).whenPressed()
        //Driver Right
        new JoystickButton(ControlMap.DRIVER_RIGHT, 1)
                .toggleWhenPressed(new IntakeDeployToggle());
        new JoystickButton(ControlMap.DRIVER_RIGHT, 2)
                .toggleWhenPressed(new CollectToggle(false, true, true));

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
                .whenPressed(new ClimbToggle());
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 8)
                .whenPressed(new InstantCommand(() -> {
                    try {
                        CommandScheduler.getInstance().cancel(CommandScheduler.getInstance().requiring(Climb.getInstance()));
                    } catch (Exception e) {
                    }
                    Climb.getInstance().setVoltage(0);
                }));
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

        //CO-DRIVER BUTTONS

        //CO-DRIVER JOYSTICK
//        new JoystickButton(ControlMap.CO_DRIVER_LEFT,1).whenPressed()

        //CO-DRIVER BUTTONS
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).whenPressed(new AutoAimShoot());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2).whenPressed(new Shoot(Floppas.ShooterLocations.FENDER));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 3).whenPressed(new Shoot(Floppas.ShooterLocations.LOW_GOAL));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).whileHeld();
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).whenPressed(new ClimbSetAngle(0));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1).whenPressed(new ClimbSetAngle(0));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 7).whenPressed(new ClimbSetAngle(0));
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
