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

package org.rivierarobotics.util.aifield;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import org.rivierarobotics.lib.shuffleboard.RSTab;
import org.rivierarobotics.robot.Logging;
import org.rivierarobotics.robot.Robot;
import org.rivierarobotics.subsystems.swervedrive.DriveTrain;

import java.awt.Polygon;
import java.awt.geom.Line2D;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;


/**
 * Constructs a Field Mesh using parameters found inside the fieldInfo.txt and fieldObstacles.txt files.
 * A* Pathfinding is used to find Trajectories from point a to point b on the mesh, and the mesh supports weighted paths
 * and object avoidance.
 */
public class FieldMesh {
    private static FieldMesh fieldMesh;

    public static FieldMesh getInstance() throws FileNotFoundException {
        if (fieldMesh == null) {
            fieldMesh = new FieldMesh();
        }
        return fieldMesh;
    }

    public int fieldWidth;
    public int fieldHeight;
    private int aiResolution;
    private final RSTab tab;
    private static final List<Polygon> FIELD_OBSTACLES = new ArrayList<>();
    private static final List<ArrayList<FieldNode>> NODES = new ArrayList<>();
    private static final List<AreaWeight> AREA_WEIGHTS = new ArrayList<>();
    public static final double MAX_VELOCITY = 3.0;
    public static final double MAX_ACCELERATION = 0.4;
    public static final double DEFAULT_NODE_WEIGHT = 0;
    private double totalTimePassed = 0;
    private double amtOfCalculations = 1;

    private FieldMesh() throws FileNotFoundException {
        Path fieldDimension = Filesystem.getDeployDirectory().toPath().resolve("AI/fieldInfo.txt");
        Scanner sc = new Scanner(new FileReader(fieldDimension.toFile()));

        sc.next();
        fieldWidth = (int) (sc.nextDouble() * 100);
        sc.next();
        fieldHeight = (int) (sc.nextDouble() * 100);
        sc.next();
        aiResolution = (int) (sc.nextDouble());
        sc.close();

        Pattern parseInput = Pattern.compile("\\(([^)]+)\\)");

        Path fieldObstacles = Filesystem.getDeployDirectory().toPath().resolve("AI/fieldObstacles.txt");
        sc = new Scanner(fieldObstacles.toFile());

        sc.nextLine();
        while (sc.hasNextLine()) {
            String obstacle = sc.nextLine();
            Matcher m = parseInput.matcher(obstacle);
            Polygon p = new Polygon();
            while (m.find()) {
                String[] in = m.group(1).split(",");
                p.addPoint(Integer.parseInt(in[0].trim()), Integer.parseInt(in[1].trim()));
            }
            FieldMesh.FIELD_OBSTACLES.add(p);
        }
        sc.close();

        Path fieldWeights = Filesystem.getDeployDirectory().toPath().resolve("AI/fieldWeights.txt");
        sc = new Scanner(fieldWeights.toFile());

        sc.nextLine();

        while (sc.hasNextLine()) {
            double wt = sc.nextDouble();
            sc.nextLine();
            var m = parseInput.matcher(sc.nextLine());
            m.find();
            String[] in = m.group(1).split(",");
            m.find();
            String[] in2 = m.group(1).split(",");

            AREA_WEIGHTS.add(
                    new AreaWeight(
                            wt,
                            Integer.parseInt(in[0]),
                            Integer.parseInt(in[1]),
                            Integer.parseInt(in2[0]),
                            Integer.parseInt(in2[1])
                    ));
        }

        sc.close();

        tab = Logging.robotShuffleboard.getTab("AI");

        tab.setEntry("AI Resolution (cm)", aiResolution);

        updateFieldMesh();
    }

    /**
     * Creates a field mesh of nodes, mesh is represented as a graph where each
     * node is set as a neighbor of adjacent nodes.
     */
    private void updateFieldMesh() {
        NODES.clear();
        int cnt = 0;
        for (int i = 0; i <= fieldHeight; i += aiResolution) {
            NODES.add(new ArrayList<>());
            for (int j = 0; j <= fieldWidth; j += aiResolution) {
                var nToAdd = new FieldNode(j, i);
                nToAdd.isValid = isValidPoint(nToAdd);
                NODES.get(cnt).add(nToAdd);
            }
            cnt++;
        }
        for (int i = 0; i < NODES.size(); i++) {
            for (int j = 0; j < NODES.get(i).size(); j++) {
                if (!NODES.get(i).get(j).isValid) {
                    continue;
                }
                weightNode(NODES.get(i).get(j));
                for (int a = Math.max(0, i - 1); a <= Math.min(i + 1, NODES.size() - 1); a++) {
                    for (int b = Math.max(0, j - 1); b <= Math.min(j + 1, NODES.get(i).size() - 1); b++) {
                        if ((i == a && j == b) || !NODES.get(a).get(b).isValid) {
                            continue;
                        }
                        if (!isValidEdge(NODES.get(i).get(j), NODES.get(a).get(b))) {
                            continue;
                        }
                        int xDist = i - a;
                        int yDist = j - b;
                        NODES.get(i).get(j).addBranch(Math.sqrt(xDist * xDist + yDist * yDist), NODES.get(a).get(b));
                    }
                }
            }
        }
    }

    /**
     * Adds an obstacle to the field.
     *
     * @param values list of points {double x, double y}(cm) that correspond to a polygons location
     */
    public void addObstacle(final ArrayList<int[]> values) {
        Polygon p = new Polygon();
        for (var pt : values) {
            p.addPoint(pt[0], pt[1]);
        }
        FIELD_OBSTACLES.add(p);
        updateFieldMesh();
    }

    /**
     * Removes an obstacle from the field.
     *
     * @param obstacle the obstacle to remove
     */
    public void removeObstacle(Polygon obstacle) {
        FIELD_OBSTACLES.remove(obstacle);
        updateFieldMesh();
    }

    public int getAiResolution() {
        return aiResolution;
    }

    /**
     * Sets the resolution of the grid (lower is more accurate).
     * for a grid of size 1m by 1m, a resolution of 10 would produce 100 nodes (10*10).
     *
     * @param resolution resolution for the grid
     */
    public void setAiResolution(int resolution) {
        this.aiResolution = resolution;
        updateFieldMesh();
    }

    public void weightNode(FieldNode node) {
        node.nodeWeight = DEFAULT_NODE_WEIGHT;
        for (var aw : AREA_WEIGHTS) {
            if (aw.containsNode(node)) {
                node.nodeWeight += aw.weight;
            }
        }
    }

    public List<AreaWeight> getAreaWeights() {
        return AREA_WEIGHTS;
    }

    /**
     * Add weights to the nodes in an area to penalize traveling through it. negative weights will
     * prioritize a zone more than others.
     *
     * @param weight amount of weight you want to add to a node (default is 1)
     * @param x1     top left x location of zone (cm)
     * @param y1     top left y location of zone (cm)
     * @param x2     bottom right x location of zone (cm)
     * @param y2     bottom right y location of zone (cm)
     */
    public void addWeightToArea(double weight, double x1, double y1, double x2, double y2) {
        int cNodeX1 = MathUtil.clamp((int) (x1 / aiResolution), 0, NODES.get(0).size() - 1);
        int cNodeY1 = MathUtil.clamp((int) (y1 / aiResolution), 0, NODES.size() - 1);
        int cNodeX2 = MathUtil.clamp((int) (x2 / aiResolution), 0, NODES.get(0).size() - 1);
        int cNodeY2 = MathUtil.clamp((int) (y2 / aiResolution), 0, NODES.size() - 1);
        for (int i = cNodeX1; i <= cNodeX2; i++) {
            for (int j = cNodeY1; j <= cNodeY2; j++) {
                NODES.get(j).get(i).nodeWeight += weight;
            }
        }
    }

    public boolean isValidPoint(FieldNode s) {
        for (var obstacle : FIELD_OBSTACLES) {
            if (obstacle.contains(s.xValue, s.yValue)) {
                return false;
            }
        }
        return true;
    }

    public boolean isValidEdge(FieldNode to, FieldNode from) {
        Line2D nodeLines = new Line2D.Double();
        nodeLines.setLine(to.xValue, to.yValue, from.xValue, from.yValue);
        for (var obstacle : FIELD_OBSTACLES) {
            for (int i = 1; i < obstacle.npoints; i++) {
                Line2D testIntersect = new Line2D.Double();
                testIntersect.setLine(
                        obstacle.xpoints[i],
                        obstacle.ypoints[i],
                        obstacle.xpoints[i - 1],
                        obstacle.ypoints[i - 1]);
                if (testIntersect.intersectsLine(nodeLines)) {
                    return false;
                }
            }
        }
        return true;
    }

    public List<Polygon> getObstacles() {
        return FIELD_OBSTACLES;
    }

    private double angleBetweenNodes(FieldNode a, FieldNode b) {
        return Math.atan2(b.yValue - a.yValue, b.xValue - a.xValue);
    }

    private long startTime = 0;

    /**
     * Trajectory Generator which utilizes A* Pathfinding to generate a trajectory that avoids
     * any obstacles on the field.
     *
     * @param x1              start x value (m)
     * @param y1              start y value (m)
     * @param x2              end x value (m)
     * @param y2              end y value (m)
     * @param shouldStop      whether the path should stop at 0 m/s or continue at max velocity
     * @param initialVelocity initial velocity of robot. useful for periodic path generation
     * @return Trajectory which corresponds to the start and end values on the field. Avoid objects specified in fieldObstacles.txt
     */
    public Trajectory getTrajectory(double x1, double y1, double x2, double y2, boolean shouldStop, double initialVelocity) {
        try {
            startTime = System.nanoTime();
            List<FieldNode> path = getPath(x1 * 100.0, y1 * 100.0, x2 * 100.0, y2 * 100.0);
            var poseList = convertPathToPose2d(path);
            if (poseList == null || poseList.size() <= 1) {
                return null;
            }

            TrajectoryConfig config = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);
            config.setEndVelocity(shouldStop ? 0 : MAX_VELOCITY);
            config.setStartVelocity(initialVelocity);
            if (Robot.isReal()) {
                config.setKinematics(DriveTrain.getInstance().getSwerveDriveKinematics());
            }

            Trajectory trajectory = null;
            try {
                trajectory = TrajectoryGenerator.generateTrajectory(
                        poseList.get(0),
                        poseList.size() >= 2 ? poseList.subList(1, poseList.size() - 2).stream().map(Pose2d::getTranslation).collect(Collectors.toList()) : new ArrayList<>(),
                        poseList.get(poseList.size() - 1),
                        config
                );
            } catch (Exception e) {
                e.printStackTrace();
                return null;
            }

            totalTimePassed += (System.nanoTime() - startTime) / 1e7;
            if (totalTimePassed < 0) {
                totalTimePassed = 0;
                amtOfCalculations = 0;
            }
            amtOfCalculations++;
            double avgTime = totalTimePassed / amtOfCalculations;
            tab.setEntry("Avg Calculation Time (ms)", (avgTime));

            return trajectory;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return null;
    }

    private List<Pose2d> convertPathToPose2d(List<FieldNode> path) {
        if (path == null || path.size() <= 1) {
            return null;
        }
        List<Pose2d> poseList = new ArrayList<>();
        var prev = path.get(0);
        poseList.add(new Pose2d(new Translation2d(path.get(0).xValue / 100.0, path.get(0).yValue / 100.0), new Rotation2d(angleBetweenNodes(path.get(0), path.get(1)))));
        int cnt = 0;
        for (FieldNode n : path) {
            cnt++;
            double angle = angleBetweenNodes(prev, n);
            prev = n;
            if (cnt == 1) {
                continue;
            }
            poseList.add(new Pose2d(new Translation2d(n.xValue / 100.0, n.yValue / 100.0), new Rotation2d(angle)));
        }
        return poseList;
    }

    /**
     * utilizes A* Pathfinding to return the shortest path to the target while avoiding obstacles.
     *
     * @param x1 start x value (cm)
     * @param y1 start y value (cm)
     * @param x2 end x value (cm)
     * @param y2 end y value (cm)
     * @return list of mesh nodes which belong to the path
     */
    public List<FieldNode> getPath(double x1, double y1, double x2, double y2) {
        int cNodeX1 = MathUtil.clamp((int) (x1 / aiResolution), 0, NODES.get(0).size() - 1);
        int cNodeY1 = MathUtil.clamp((int) (y1 / aiResolution), 0, NODES.size() - 1);
        int cNodeX2 = MathUtil.clamp((int) (x2 / aiResolution), 0, NODES.get(0).size() - 1);
        int cNodeY2 = MathUtil.clamp((int) (y2 / aiResolution), 0, NODES.size() - 1);
        FieldNode start = NODES.get(cNodeY1).get(cNodeX1);
        FieldNode end = NODES.get(cNodeY2).get(cNodeX2);
        var n = findShortestPath(start, end);
        return printPath(n);
    }

    /**
     * gets the field nodes.
     *
     * @return All the nodes that make up the field mesh
     */
    public List<ArrayList<FieldNode>> getFieldNodes() {
        return NODES;
    }

    /**
     * Helper utility for A* Algorithm to retrieve the shortest path after calculation.
     */
    public List<FieldNode> printPath(FieldNode target) {
        FieldNode n = target;
        if (n == null) {
            return null;
        }
        List<FieldNode> ids = new ArrayList<>();
        while (n.parent != null) {
            ids.add(n);
            n = n.parent;
        }
        ids.add(n);
        Collections.reverse(ids);
        return ids;
    }

    /**
     * have fun :)
     * https://stackabuse.com/graphs-in-java-a-star-algorithm/
     */
    public FieldNode findShortestPath(FieldNode start, FieldNode target) {
        if (!start.isValid || !target.isValid) {
            return null;
        }

        for (var n : NODES) {
            for (var p : n) {
                p.f = Double.MAX_VALUE;
                p.g = Double.MAX_VALUE;
                p.parent = null;
            }
        }

        PriorityQueue<FieldNode> closedList = new PriorityQueue<>();
        PriorityQueue<FieldNode> openList = new PriorityQueue<>();
        start.g = 0;
        start.f = start.g + start.calculateHeuristic(target);
        openList.add(start);

        while (!openList.isEmpty()) {
            FieldNode n = openList.peek();
            if (n == target) {
                return n;
            }

            for (FieldNode.Edge edge : n.neighbors) {
                FieldNode m = edge.node;
                double totalWeight = n.g + edge.weight + edge.node.nodeWeight;
                if (!openList.contains(m) && !closedList.contains(m)) {
                    m.parent = n;
                    m.g = totalWeight;
                    m.f = m.g + m.calculateHeuristic(target);
                    openList.add(m);
                } else {
                    if (totalWeight < m.g) {
                        m.parent = n;
                        m.g = totalWeight;
                        m.f = m.g + m.calculateHeuristic(target);

                        if (closedList.contains(m)) {
                            closedList.remove(m);
                            openList.add(m);
                        }
                    }
                }
            }

            openList.remove(n);
            closedList.add(n);
        }
        return null;
    }

}
