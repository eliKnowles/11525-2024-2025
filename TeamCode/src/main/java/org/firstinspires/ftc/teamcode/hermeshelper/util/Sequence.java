package org.firstinspires.ftc.teamcode.hermeshelper.util;

import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.DcMotorV2;
import org.firstinspires.ftc.teamcode.hermeshelper.util.hardware.ServoV2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Sequence {
    // Inner class to represent a command with its target, position, and delay
    private static class Command {
        Object target;
        double position;
        int delay;

        public Command(Object target, double position, int delay) {
            this.target = target;
            this.position = position;
            this.delay = delay;
        }

        public void execute() {
            // Add specific handling for different types of objects (e.g., Servo, DcMotor)
            if (target instanceof ServoV2) {
                ((ServoV2) target).setPosition(position);
            } else if (target instanceof DcMotorV2) {
                ((DcMotorV2) target).runToPosition((int) position);
            }
            // Add handling for other types if needed
            try {
                Thread.sleep(delay);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    // Store sequences by name
    private final Map<String, List<Command>> sequences = new HashMap<>();
    private List<Command> currentSequence;

    // Method to create a new sequence by name
    public Sequence create(String name) {
        currentSequence = new ArrayList<>();
        sequences.put(name, currentSequence);
        return this;
    }

    // Method to add a command to the current sequence
    public Sequence add(Object target, double position, int delay) {
        if (currentSequence != null) {
            currentSequence.add(new Command(target, position, delay));
        }
        return this;
    }

    // Method to finalize the current sequence
    public void build() {
        currentSequence = null;
    }

    // Method to run a sequence by name
    public void run(String name) {
        List<Command> sequence = sequences.get(name);
        if (sequence != null) {
            for (Command command : sequence) {
                command.execute();
            }
        } else {
            System.out.println("Sequence not found: " + name);
        }
    }
}
