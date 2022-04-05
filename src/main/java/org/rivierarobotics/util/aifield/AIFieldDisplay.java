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

package org.rivierarobotics.util.aifield;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Adds the "AI Mesh" Camera to the CameraServer which is able to be updated with paths as the robot calculates them.
 * The Thread is only here for testing purposes only, Updating the path should be done by calling updatePath
 */
public class AIFieldDisplay {
    private static final int SCALING_FACTOR = 1000;
    private final ScheduledExecutorService mainImageThread = Executors.newSingleThreadScheduledExecutor();
    private final AtomicReference<Mat> renderFrame = new AtomicReference<>();
    private final CvSource outputStream;
    private final FieldMesh fieldMesh;
    private final int imgWidth;
    private final int imgHeight;
    private final double scalingRatio;
    private Trajectory generatedTrajectory;
    private Mat fieldMat;
    private int tick;

    public AIFieldDisplay(int updateRate, boolean fieldThread) {
        this.imgHeight = SCALING_FACTOR;
        this.fieldMesh = FieldMesh.getInstance();
        this.imgWidth = (int) (SCALING_FACTOR * (((double) fieldMesh.fieldWidth) / fieldMesh.fieldHeight));
        this.scalingRatio = (double) SCALING_FACTOR / fieldMesh.fieldHeight;
        this.outputStream = CameraServer.putVideo("AI Mesh", imgWidth, imgHeight);
        outputStream.setResolution(480, 480);
        updatePath(fieldMesh.getTrajectory(0, 0, 5, 5, true, 0.1, DriveTrain.getInstance().getSwerveDriveKinematics()));
        updateField();
        if (fieldThread) {
            if (!DriverStation.isFMSAttached()) {
                startFieldThread(updateRate);
            }
        }
    }

    private void startFieldThread(int updateRate) {
        int size = 480;
        Mat resizeFrame = new Mat(size, (int) (size * scalingRatio), CvType.CV_8UC(4), Scalar.all(100));
        mainImageThread.scheduleWithFixedDelay(() -> {
            Mat image = renderFrame.getOpaque();
            Imgproc.resize(image, resizeFrame, resizeFrame.size(), 0, 0, 2);
            if (!Robot.isReal()) {
                outputStream.putFrame(image);
            } else {
                outputStream.putFrame(resizeFrame);
            }
        }, 0, updateRate, TimeUnit.MILLISECONDS);
    }

    private void render() {
        if (generatedTrajectory == null || fieldMat == null) {
            return;
        }
        var newRenderFrame = fieldMat.clone();
        var trajectory = generatedTrajectory;
        mainImageThread.submit(() -> {
            for (double i = 0; i < trajectory.getTotalTimeSeconds(); i += 0.1) {
                if (trajectory.getTotalTimeSeconds() < i + 0.1) {
                    continue;
                }
                var pose1 = trajectory.sample(i);
                var pose2 = trajectory.sample(i + 0.1);

                Imgproc.arrowedLine(
                    newRenderFrame,
                    new Point(pose1.poseMeters.getX() * 100 * scalingRatio, pose1.poseMeters.getY() * 100 * scalingRatio),
                    new Point(pose2.poseMeters.getX() * 100 * scalingRatio, pose2.poseMeters.getY() * 100 * scalingRatio),
                    new Scalar(0, 0, 255 * (trajectory.sample(i).velocityMetersPerSecond) / 2),
                    10
                );
            }
            Core.flip(newRenderFrame, newRenderFrame, 0);
            renderFrame.setOpaque(newRenderFrame);
        });
    }

    public void updatePath(Trajectory trajectory) {
        if (trajectory == null) {
            return;
        }
        this.generatedTrajectory = Objects.requireNonNull(trajectory);
        render();
    }

    public void updateField() {
        this.fieldMat = createField();
        render();
    }

    private Mat createField() {
        Mat field = new Mat(imgHeight, imgWidth, CvType.CV_8UC(4), Scalar.all(100));

        drawFieldMesh(field);
        drawFieldObstacles(field);
        drawWeightedAreas(field);

        return field;
    }

    private void drawFieldObstacles(Mat field) {
        Scalar color = new Scalar(64, 64, 64);
        int lineType = Imgproc.LINE_8;
        var fieldObstacles = fieldMesh.getObstacles();
        for (var obstacle : fieldObstacles) {
            List<MatOfPoint> list = new ArrayList<>();
            Point[] pts = new Point[obstacle.npoints];
            for (int i = 0; i < obstacle.npoints; i++) {
                pts[i] = (new Point(obstacle.xpoints[i] * scalingRatio, obstacle.ypoints[i] * scalingRatio));
            }
            list.add(new MatOfPoint(pts));
            Imgproc.fillPoly(field, list, color, lineType);
        }
    }

    private void drawWeightedAreas(Mat field) {
        var weightedAreas = fieldMesh.getAreaWeights();
        for (var aw : weightedAreas) {
            Imgproc.rectangle(
                field,
                new Point(aw.x1 * scalingRatio, aw.y1 * scalingRatio),
                new Point(aw.x2 * scalingRatio, aw.y2 * scalingRatio),
                new Scalar(0, 0, 255 * (aw.weight / 20.0)),
                3);
        }
    }

    private void drawFieldMesh(Mat field) {
        var nodes = fieldMesh.getFieldNodes();
        for (var n : nodes) {
            for (var p : n) {
                if (!p.isValid) {
                    continue;
                }
                for (var e : p.neighbors) {
                    Point a = new Point(p.xValue * scalingRatio, p.yValue * scalingRatio);
                    Point b = new Point(e.node.xValue * scalingRatio, e.node.yValue * scalingRatio);
                    if (p.nodeWeight < 0) {
                        Imgproc.line(field, a, b, new Scalar(255, 0, 0, 1));
                    } else {
                        Imgproc.line(field, a, b, new Scalar(0, 255 - (255 * (p.nodeWeight) / 60.0), 255 * (p.nodeWeight) / 60.0), 1);
                    }
                }
            }
        }
    }

    /**
     * Make some visible changes to the field.
     */
    public void update() {
        tick++;
        if (tick % 50 == 0) {
            int baseX = ThreadLocalRandom.current().nextInt(fieldMesh.fieldWidth);
            int baseY = ThreadLocalRandom.current().nextInt(fieldMesh.fieldHeight);
            fieldMesh.addWeightToArea(50, baseX, baseY, baseX + 100, baseY + 100);
            updateField();
        }
    }
}
