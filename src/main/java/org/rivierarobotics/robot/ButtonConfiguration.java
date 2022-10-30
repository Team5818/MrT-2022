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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.rivierarobotics.commands.advanced.climb.ClimbInterruptToggle;
import org.rivierarobotics.commands.advanced.climb.RunClimb;
import org.rivierarobotics.commands.advanced.collect.CollectBalls;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShoot;
import org.rivierarobotics.commands.advanced.shoot.AutoAimShootEject;
import org.rivierarobotics.commands.advanced.shoot.FenderShot;
import org.rivierarobotics.commands.advanced.shoot.RotateBall;
import org.rivierarobotics.commands.advanced.shoot.TrackGoal;
import org.rivierarobotics.commands.auto.CrazyWildCollectionBouncyHousePath;
import org.rivierarobotics.commands.auto.FindBall;
import org.rivierarobotics.commands.auto.MLCollect1;
import org.rivierarobotics.commands.basic.climb.ClimbSetAngle;
import org.rivierarobotics.commands.basic.climb.ClimbSetVoltage;
import org.rivierarobotics.commands.basic.climb.SetPiston;
import org.rivierarobotics.commands.basic.collect.ToggleIntakeState;
import org.rivierarobotics.commands.basic.drive.SetDriveAngle;
import org.rivierarobotics.commands.basic.drive.SetDriverAssist;
import org.rivierarobotics.commands.basic.shoot.SetFloppaZero;
import org.rivierarobotics.commands.basic.shoot.SetFlywheelSpeed;
import org.rivierarobotics.subsystems.climb.Climb;
import org.rivierarobotics.subsystems.climb.ClimbPositions;
import org.rivierarobotics.subsystems.shoot.FloppaFlywheels;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;
import org.rivierarobotics.subsystems.vision.Limelight;
import org.rivierarobotics.util.Gyro;

public class   ButtonConfiguration {
    public void initTeleop() {
        // DRIVER LEFT
//move to codriver
//        new JoystickButton(ControlMap.DRIVER_LEFT, 1)
//            .toggleWhenPressed(new ClimbSetAngle(0, false));

        new JoystickButton(ControlMap.DRIVER_LEFT, 1)
                .whenPressed(new SetDriveAngle(0));
        new JoystickButton(ControlMap.DRIVER_LEFT, 2)
                .whenPressed(new SetDriveAngle(180));

        // DRIVER RIGHT
        //good
        new JoystickButton(ControlMap.DRIVER_RIGHT, 1)
            .whenPressed(new ToggleIntakeState());
        new JoystickButton(ControlMap.DRIVER_RIGHT, 2)
            .toggleWhenPressed(new CollectBalls());

        // DRIVER BUTTONS

        //good
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 1)
            .whenPressed(new AutoAimShootEject(false));
        //good
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 2)
            .whenPressed(new AutoAimShoot(false));
        //good
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 3)
            .whenPressed(new FenderShot());

        new JoystickButton(ControlMap.DRIVER_BUTTONS, 7)
                .toggleWhenPressed(new InstantCommand(() ->
                        DriveTrain.getInstance().setTargetRotationAngle(135)));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 9)
                .whenPressed(new RunClimb(false));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 11)
                .whenPressed(new InstantCommand(() -> Gyro.getInstance().resetGyro()));
//move to codriver
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 7)
//            .whenPressed(new ClimbInterruptToggle());
//        new JoystickButton(ControlMap.DRIVER_BUTTONS, 8)
//            .whenPressed(new ClimbSetVoltage(false, 0));


        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13)
            .whileHeld(new SetDriverAssist(true));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 13)
            .whenReleased(new SetDriverAssist(false));

        new JoystickButton(ControlMap.DRIVER_BUTTONS, 14)
                .whenHeld(new PowerSave(true));
        new JoystickButton(ControlMap.DRIVER_BUTTONS, 14)
                .whenReleased(new PowerSave(false));

        // CO-DRIVER LEFT
        new JoystickButton(ControlMap.CO_DRIVER_RIGHT, 1)
                .whenPressed(new ToggleIntakeState());
        new JoystickButton(ControlMap.CO_DRIVER_RIGHT, 2)
                .whenPressed(new CollectBalls());
        // CO-DRIVER RIGHT

        // CO-DRIVER BUTTONS
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 1)
                .whenPressed(new AutoAimShootEject(false));
        //good
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 2)
                .whenPressed(new AutoAimShoot(false));
        //good
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 3)
                .whenPressed(new FenderShot());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 4)
                .whenPressed(new SetFloppaZero());
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 5)
                .whenPressed(new RunClimb(false));
        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 6)
                .whenPressed(new ClimbInterruptToggle());

//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 7)
//            .whenPressed(new SetPiston(ClimbPositions.LOW, true));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8)
//            .whenPressed(new SetPiston(ClimbPositions.LOW, false));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 9)
//            .whenPressed(new SetPiston(ClimbPositions.MID, true));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 10)
//            .whileHeld(new SetPiston(ClimbPositions.MID, false));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 11)
//            .whenPressed(new SetPiston(ClimbPositions.HIGH, true));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 12)
//            .whenPressed(new SetPiston(ClimbPositions.HIGH, false));

//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8)
//            .whileHeld(new SequentialCommandGroup(new SetDriveAngle(77)));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 8)
//            .whenPressed(new InstantCommand(() ->
//                    DriveTrain.getInstance().getPoseEstimator().resetPose(Limelight.getInstance().getLLAbsPose()))
//        );
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 9)
//            .whenPressed(() -> Climb.getInstance().resetZeros(false));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 10)
//            .whenPressed(new RotateBall());
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 12)
//            .whenPressed(new MLCollect1(false));
//        new JoystickButton(ControlMap.CO_DRIVER_BUTTONS, 15)
//            .whileHeld(new TrackGoal(false));
    }
}
