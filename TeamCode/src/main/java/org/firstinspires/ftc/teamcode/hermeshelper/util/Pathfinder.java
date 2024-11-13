package org.firstinspires.ftc.teamcode.hermeshelper.util;

import org.firstinspires.ftc.teamcode.hermeshelper.util.mechanum_drive.MechanumDrive;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class Pathfinder {
    private MechanumDrive drive;
    private int[][] board;
    private boolean[][] visited;
    private int rows, cols;

    private final int[][] DIRECTIONS = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    public Pathfinder(MechanumDrive drive, int[][] board) {
        this.drive = drive;
        this.board = board;
        this.rows = board.length;
        this.cols = board[0].length;
        this.visited = new boolean[rows][cols];
    }

    private class Node {
        int x, y;
        LinkedList<int[]> path;

        public Node(int x, int y, LinkedList<int[]> path) {
            this.x = x;
            this.y = y;
            this.path = new LinkedList<>(path);
            this.path.add(new int[] {x, y});
        }
    }

    public void genPathAndMove(int startX, int startY, int endX, int endY){
        Queue<Node> queue = new LinkedList<>();
        queue.add(new Node(startX, startY, new LinkedList<>()));
        visited[startX][startY] = true;
        while (!queue.isEmpty()) {
            Node current = queue.poll();

            if (current.x == endX && current.y == endY) {
                moveSpline(current.path);
                return;
            }


            for (int[] dir : DIRECTIONS) {
                int newX = current.x + dir[0];
                int newY = current.y + dir[1];


                if (isValid(newX, newY) && !visited[newX][newY]) {
                    visited[newX][newY] = true;
                    queue.add(new Node(newX, newY, current.path));
                }
            }
        }
    }

    private void moveSpline(LinkedList<int[]> path){
        List<double[]> splinePath = generateSpline(path);

        for (double[] point : splinePath) {
            double targetX = point[0];
            double targetY = point[1];

            double driveX = targetX;
            double driveY = targetY;

            drive.drive(driveX, driveY, 0);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        
        drive.drive(0, 0, 0);
    }

    private List<double[]> generateSpline(LinkedList<int[]> path){
        List<double[]> splinePoints = new ArrayList<>();

        for (int i = 1; i < path.size(); i++) {
            int[] start = path.get(i - 1);
            int[] end = path.get(i);

            double step = 0.1;
            for (double t = 0; t <= 1; t += step) {
                double x = (1 - t) * start[1] + t * end[1];
                double y = (1 - t) * start[0] + t * end[0];
                splinePoints.add(new double[] {x, y});
            }
        }

        return splinePoints;
    }

    private boolean isValid(int x, int y) {
        return x >= 0 && x < rows && y >= 0 && y < cols && board[x][y] == 1;
    }
}