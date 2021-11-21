package edu.gatech.cs.mdp.experiments;

import java.io.IOException;

import edu.gatech.cs.mdp.problems.GridWorldProblem;

public class LargeMaze {
    GridWorldProblem gwp;

    public LargeMaze(){
        this.gwp = new GridWorldProblem("maze9.csv", 0.8, 50.0, -3.0, -1.0);
    }
    
    public void runGammaExperiment() {

        double [] gammas = new double[] {0.99, 0.999, 0.9999};
        int[] minIters = new int[] {195, 170, 170};
        for (int i=0; i<gammas.length; i++) { 
            if (i != 1) {
                continue;
            }
            try {
                gwp.valueIterationExperimenter(
                    "output_maze9_" + i + "/",
                    new double[] {gammas[i]}, minIters[i], 1, 10000000, 10, Math.pow(10,-7));
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }

    public void runGammaVIExperimentJustPlanning() {

        try {
            gwp.valueIterationMeanV("output_maze9_2/", new double[] {0.99, 0.999, 0.9999}, 1, 1, 100, 1, Math.pow(10,-5));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void runGammaPIExperiment() {

        try {
            gwp.policyIterationExperimenter("output_maze9_2/", new double[] {0.99, 0.999, 0.9999}, 10, 1, 100, 5, Math.pow(10,-5));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void runGammaPIExperimentJustPlanning() {

        try {
            gwp.policyIterationMeanV("output_maze9_2/", new double[] {0.99, 0.999, 0.9999}, 1, 1, 100, 1, Math.pow(10,-5));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    public static void main(String[] args) {
        LargeMaze sm = new LargeMaze();
        //sm.runGammaExperiment();
        sm.runGammaVIExperimentJustPlanning();
        // sm.runGammaPIExperiment();
        // sm.runGammaPIExperimentJustPlanning();
    }
}
