package edu.gatech.cs.mdp.experiments;

import java.io.IOException;

import edu.gatech.cs.mdp.problems.BlockDudeProblem;

public class LargeBlockDude {
    BlockDudeProblem bwp;

    public LargeBlockDude(){
        this.bwp = new BlockDudeProblem(1);
    }
    
    public void runGammaVIExperiment() {

        try {
            bwp.valueIterationExperimenter("output_large_blockdude/", new double[] {0.98, 0.99, 0.999, 0.9999}, 100, 5, 10000, 10, Math.pow(10,-1));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void runGammaVIExperimentJustPlanning() {

        try {
            bwp.valueIterationMeanV("output_large_blockdude/", new double[] {0.98, 0.99, 0.999, 0.9999}, 1, 5, 10000, 1, Math.pow(10,-1));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void runGammaPIExperiment() {

        try {
            bwp.policyIterationExperimenter("output_large_blockdude/", new double[] {0.98, 0.99, 0.999, 0.9999}, 3, 1, 100, 10, Math.pow(10,-1));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
/*
    public void runGammaPIExperimentJustPlanning() {

        try {
            bwp.policyIterationMeanV("output_large_blockdude/", new double[] {0.9999}, 1, 1, 100, 1, Math.pow(10,-5));
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
    */

    public static void main(String[] args) {
        LargeBlockDude sm = new LargeBlockDude();
        // sm.runGammaVIExperiment();
        sm.runGammaVIExperimentJustPlanning();
        // sm.runGammaPIExperiment();
        // sm.runGammaPIExperimentJustPlanning();
    }
}
