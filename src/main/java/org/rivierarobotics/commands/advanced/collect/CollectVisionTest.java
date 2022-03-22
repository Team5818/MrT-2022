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

package org.rivierarobotics.commands.advanced.collect;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.util.ml.BoundingBox;
import org.rivierarobotics.util.ml.MLCore;
import org.rivierarobotics.util.ml.MLObject;

public class CollectVisionTest extends CommandBase {
//    public MLObject ball;
//    public final MLCore mlCore;
    public final BoundingBox defaultBallBox = new BoundingBox(0, 0, 0, 0);

    public CollectVisionTest() {
//        this.mlCore = MLCore.getInstance();
//        this.ball = new MLObject("red", defaultBallBox,1);
//
//        try {
//            this.ball = mlCore.getDetectedObjects().get("red").get(0);
//        } catch (NullPointerException nullPointerException){
//            return;
//        }

    }

    @Override
    public void initialize() {
        MLCore core = MLCore.getInstance();
        MLObject ball = new MLObject("red", defaultBallBox, 1);

        try {
            ball = core.getDetectedObjects().get("red").get(0);
        } catch (NullPointerException ignored) {
            // Padding for checkstyle
        }

        Logging.robotShuffleboard.getTab("ML").setEntry("Target BallX", ball.relativeLocationY)
                .setEntry("Target BallY", ball.relativeLocationX)
                .setEntry("Target Ball Distance", ball.relativeLocationDistance)
                .setEntry("TX", ball.ty)
                .setEntry("TY", ball.tx);
    }
}
