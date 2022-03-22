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

package org.rivierarobotics.util.ml;

public class MLObject {
    public static final double BALL_RADIUS = 2 * 0.12;
    public static final double CAMERA_OFFSET = 0.775 - BALL_RADIUS;

    public final String label;
    public final BoundingBox boundingBox;
    public final double ty;
    public final double tx;
    public final double relativeLocationY;
    public final double relativeLocationX;
    public final double relativeLocationDistance;
    public final double confidence;
//    private final Map<Double, Double> yAngle = new HashMap<Double, Double>();
//    private final Map<Double, Double> xAngle = new HashMap<Double, Double>();

    public MLObject(String label, BoundingBox boundingBox, double confidence) {
        this.label = label;
        this.boundingBox = boundingBox;
        // Use the bounding box Y value to compute our field X value
        double avgX = (boundingBox.ymax + boundingBox.ymin) / 2.0;
        this.tx = (MLCore.CAMERA_HEIGHT / 2.0 - avgX) * MLCore.ANGLE_PER_PIXEL_Y;
        this.relativeLocationX = Math.tan((90 - Math.abs(tx)) * Math.PI / 180) * CAMERA_OFFSET;

        double avgY = (boundingBox.xmax + boundingBox.xmin) / 2.0;
        this.ty = (MLCore.CAMERA_WIDTH / 2.0 - avgY) * MLCore.ANGLE_PER_PIXEL_X;
        this.relativeLocationY = relativeLocationX * Math.tan(ty * Math.PI / 180);

        this.relativeLocationDistance = Math.sqrt(Math.pow(relativeLocationY, 2) + Math.pow(relativeLocationX, 2));
        this.confidence = confidence;

//        yAngle.put(464., 36.87);
//        yAngle.put(432., 32.005);
//        yAngle.put(400., 26.565);
//        yAngle.put(361., 20.556);
//        yAngle.put(322., 14.036);
//        yAngle.put(278., 7.125);
//        yAngle.put(240., 0.);
//        yAngle.put(202., 7.125);
//        yAngle.put(158.,14.036);
//        yAngle.put(119.,20.556);
//        yAngle.put(80., 26.565);
//        yAngle.put(48., 32.005);
//        yAngle.put(16., 36.87);
//
//        xAngle.put(633., 53.972);
//        xAngle.put(616., 51.34);
//        xAngle.put(602., 48.36);
//        xAngle.put(583., 45.);
//        xAngle.put(562., 41.18);
//        xAngle.put(537., 36.87);
//        xAngle.put(509., 32.005);
//        xAngle.put(479., 26.565);
//        xAngle.put(446., 20.556);
//        xAngle.put(403., 14.036);
//        xAngle.put(365., 7.125);
//        xAngle.put(320., 0.);
//        xAngle.put(275., 7.125);
//        xAngle.put(237.,14.036);
//        xAngle.put(194.,20.556);
//        xAngle.put(161., 26.565);
//        xAngle.put(131., 32.005);
//        xAngle.put(103., 36.87);
//        xAngle.put(78., 41.18);
//        xAngle.put(57., 45.);
//        xAngle.put(38., 48.36);
//        xAngle.put(24., 51.34);
//        xAngle.put(7., 53.972);

    }



}
