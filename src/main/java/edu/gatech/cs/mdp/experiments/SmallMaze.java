package edu.gatech.cs.mdp.experiments;

import java.io.IOException;

import edu.gatech.cs.mdp.problems.GridWorldProblem;

public class SmallMaze {
    GridWorldProblem gwp;

    public SmallMaze(){
        this.gwp = new GridWorldProblem("maze6.csv", 0.8, 50.0, -3.0, -1.0);
    }
    
    public void runGammaVIExperiment() {

        try {
            gwp.valueIterationExperimenter("output_maze6_3/", new double[] {0.98, 0.99, 0.999, 0.9999}, 150, 1, 10000, 10, Math.pow(10,-5));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void runGammaPIExperiment() {

        try {
            gwp.policyIterationExperimenter("output_maze6_3/", new double[] {0.98, 0.99, 0.999, 0.9999}, 7, 1, 100, 10, Math.pow(10,-5));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void runGammaPIExperimentJustPlanning() {

        try {
            gwp.policyIterationMeanV("output_maze6_4/", new double[] {0.9999}, 1, 1, 100, 1, Math.pow(10,-5));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public static void main(String[] args) {
        SmallMaze sm = new SmallMaze();
        // sm.runGammaVIExperiment();
        // sm.runGammaPIExperiment();
        sm.runGammaPIExperimentJustPlanning();
    }
}
