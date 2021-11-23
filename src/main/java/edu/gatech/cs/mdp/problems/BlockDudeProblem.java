package edu.gatech.cs.mdp.problems;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

import com.opencsv.CSVWriter;

import burlap.behavior.policy.GreedyQPolicy;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;
import burlap.behavior.singleagent.Episode;
import burlap.behavior.singleagent.auxiliary.EpisodeSequenceVisualizer;
import burlap.behavior.singleagent.auxiliary.StateReachability;
import burlap.behavior.singleagent.auxiliary.performance.LearningAlgorithmExperimenter;
import burlap.behavior.singleagent.auxiliary.performance.PerformanceMetric;
import burlap.behavior.singleagent.auxiliary.performance.TrialMode;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.ValueFunctionVisualizerGUI;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.ArrowActionGlyph;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.LandmarkColorBlendInterpolation;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.PolicyGlyphPainter2D;
import burlap.behavior.singleagent.auxiliary.valuefunctionvis.common.StateValuePainter2D;
import burlap.behavior.singleagent.learning.LearningAgent;
import burlap.behavior.singleagent.learning.LearningAgentFactory;
import burlap.behavior.singleagent.learning.tdmethods.QLearning;
import burlap.behavior.singleagent.learning.tdmethods.SarsaLam;
import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.singleagent.planning.deterministic.DeterministicPlanner;
import burlap.behavior.singleagent.planning.deterministic.SDPlannerPolicy;
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.NullHeuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
// import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.QProvider;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.domain.singleagent.blockdude.BlockDude;
import burlap.domain.singleagent.blockdude.BlockDudeLevelConstructor;
import burlap.domain.singleagent.blockdude.BlockDudeTF;
import burlap.domain.singleagent.blockdude.BlockDudeVisualizer;
import burlap.domain.singleagent.blockdude.state.BlockDudeAgent;
import burlap.domain.singleagent.blockdude.state.BlockDudeCell;
import burlap.domain.singleagent.blockdude.state.BlockDudeMap;
import burlap.domain.singleagent.blockdude.state.BlockDudeState;
import burlap.domain.singleagent.gridworld.GridWorldDomain;
import burlap.domain.singleagent.gridworld.GridWorldRewardFunction;
import burlap.domain.singleagent.gridworld.GridWorldTerminalFunction;
import burlap.domain.singleagent.gridworld.GridWorldVisualizer;
import burlap.domain.singleagent.gridworld.state.GridAgent;
import burlap.domain.singleagent.gridworld.state.GridLocation;
import burlap.domain.singleagent.gridworld.state.GridWorldState;
import burlap.mdp.auxiliary.stateconditiontest.StateConditionTest;
import burlap.mdp.auxiliary.stateconditiontest.TFGoalCondition;
import burlap.mdp.core.TerminalFunction;
import burlap.mdp.core.state.State;
import burlap.mdp.core.state.vardomain.VariableDomain;
import burlap.mdp.singleagent.SADomain;
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;

import edu.gatech.cs.mdp.utils.PlanningUtils;
import edu.gatech.cs.mdp.burlap_custom.ValueIteration;

public class BlockDudeProblem {
	
    OOSADomain domain;
    BlockDude constructor;
    State initialState;
    TerminalFunction tf;
    StateConditionTest sc;
    SimpleHashableStateFactory hashingFactory;
    int maxx;
    int maxy;


	public BlockDudeProblem(int size) {
        constructor = new BlockDude(25, 25);
        domain = constructor.generateDomain();

		if (size == 1) {
			this.setLargeProblem();
		} else {
			this.setSmallProblem();
		}

        hashingFactory = new SimpleHashableStateFactory();
        tf = new BlockDudeTF();
        sc = new TFGoalCondition(tf);
        maxx = constructor.getMaxx();
        maxy = constructor.getMaxy();
	}

	public static void visualizeMap(int [][] map) {
		for (int j=map[0].length-1; j>-1; j--) {
			for (int i=0; i<map.length; i++) {
				System.out.print((map[i][j]));
			}
			System.out.println("");
		}
	}

	public void setSmallProblem(){

		int [][] map = new int[25][25];
		BlockDudeLevelConstructor.addFloor(map);
		BlockDudeLevelConstructor.wallSegment(map, 1, map[0].length-1, map.length-1);

		map[3][1] = 1;
		map[3][2] = 1;

		map[7][1] = 1;

		map[11][1] = 1;
		map[11][2] = 1;


		BlockDudeState s = new BlockDudeState(
				new BlockDudeAgent(15, 1, 1, false),
				new BlockDudeMap(map),
				BlockDudeCell.exit(0, 1),
				BlockDudeCell.block("b0", 9, 1),
				BlockDudeCell.block("b1", 13, 1)
		);

		this.initialState = s;
		this.visualizeMap(map);
	}

	public void setLargeProblem() {

		int [][] map = new int[25][25];

		BlockDudeLevelConstructor.floorSegment(map, 0, 1, 0);
		BlockDudeLevelConstructor.wallSegment(map, 0, 4, 1);
		BlockDudeLevelConstructor.floorSegment(map, 1, 3, 4);
		BlockDudeLevelConstructor.wallSegment(map, 0, 4, 3);
		BlockDudeLevelConstructor.floorSegment(map, 3, 8, 1);
		map[7][2] = 1;
		BlockDudeLevelConstructor.wallSegment(map, 0, 3, 8);
		BlockDudeLevelConstructor.floorSegment(map, 8, 10, 0);
		map[10][1] = 1;
		BlockDudeLevelConstructor.wallSegment(map, 1, 3, 11);
		map[12][4] = 1;
		BlockDudeLevelConstructor.floorSegment(map, 11, 15, 3);
		map[15][4] = 1;
		map[16][4] = 1;
		BlockDudeLevelConstructor.wallSegment(map, 5, 8, 17);

		BlockDudeState s = new BlockDudeState(
				new BlockDudeAgent(8, 4, 0, false),
				new BlockDudeMap(map),
				BlockDudeCell.exit(0, 1),
				BlockDudeCell.block("b0", 4, 2),
				BlockDudeCell.block("b1", 5, 2),
				BlockDudeCell.block("b2", 13, 4),
				BlockDudeCell.block("b3", 15, 5),
				BlockDudeCell.block("b4", 16, 5),
				BlockDudeCell.block("b5", 16, 6));

		this.initialState = s;
		this.visualizeMap(map);
	}
	
	public void visualize(String outputPath){

		Visualizer v = BlockDudeVisualizer.getVisualizer(this.maxx, this.maxy);
		new EpisodeSequenceVisualizer(v, domain, outputPath);
	}


	public void BFSExample(String outputPath){
		
		DeterministicPlanner planner = new BFS(domain, sc, hashingFactory);
		Policy p = planner.planFromState(initialState);
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "bfs");
			
	}


	public void DFSExample(String outputPath){
		
		DeterministicPlanner planner = new DFS(domain, sc, hashingFactory);
		Policy p = planner.planFromState(initialState);
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "dfs");
		
	}


	public void AStarExample(String outputPath){
		
        AStar astar = new AStar(domain, sc, hashingFactory, new NullHeuristic());
        // astar.toggleDebugPrinting(false);
        // System.out.println(initialState);
        astar.planFromState(initialState);

        Policy p = new SDPlannerPolicy(astar);
        Episode e = PolicyUtils.rollout(p, initialState, domain.getModel(), 100);

		e.write(outputPath + "astar");
	}

	public void valueIterationExample(String outputPath){
		
		Planner planner = new ValueIteration(domain, 0.99, hashingFactory, 0.001, 10000);
		Policy p = planner.planFromState(initialState);
	
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "vi");

		// simpleValueFunctionVis((ValueFunction)planner, p);
		
	}

	public void valueIterationMeanV(
		String outputPath,
		double[] gammaArray,
		int minIter,
		int stepSize,
		int maxIter,
		int numTrials,
		double viConvergenceCriterion
		) throws IOException{
		
		Path path = FileSystems.getDefault().getPath(outputPath);
		if (Files.notExists(path)) {
			new File(outputPath).mkdirs();
		}
			
		// setup csv
		CSVWriter writer = new CSVWriter(new FileWriter(outputPath + "vi_gamma_experiment_just_meanV.csv"));
		// write header
		writer.writeNext(new String[] {"PlanningTime (ms)", "meanV", "gamma", "maxIters", "trialNum"});

		List<Double[]> results = new ArrayList<Double[]>();
		DecimalFormat df = new DecimalFormat("#.####");
		for (double gamma : gammaArray) {
			System.out.println(gamma);
			
			// run experiment as a function of max iterations
			for (int trialNum=0; trialNum<numTrials; trialNum++){

				// convergence criteria
				double pastMeanV = Math.pow(10,10);
				int numConverged = 0;

				System.out.println("Starting run for gamma:" + gamma + " trial:" + trialNum);
				ValueIteration planner = new ValueIteration(domain, gamma, hashingFactory, viConvergenceCriterion, maxIter);
				
				Double[] experimentResultsOut = new Double[5];
				Double experimentResults = PlanningUtils.runPlanning(
					planner, domain, initialState, outputPath + "vi_gamma_" + df.format(gamma) + "_trial_" + trialNum
				);

				// prepare results
				double numIter = 0;
				for (double meanV : planner.meanVs) {
					
					experimentResultsOut[0] = experimentResults;
					experimentResultsOut[1] = meanV;
					experimentResultsOut[2] = gamma;
					experimentResultsOut[3] = numIter;
					experimentResultsOut[4] = (double) trialNum;
					
					results.add(experimentResultsOut);
					String[] stringResults = new String[5];
					for (int recordIndex=0; recordIndex < experimentResultsOut.length; recordIndex++) {
						stringResults[recordIndex] = experimentResultsOut[recordIndex].toString();
					}
					
					writer.writeNext(stringResults);
				}

			}
		}
		writer.close();
		for (Double[] result : results) {
			for (Double r : result ) {
				System.out.print(r);
				System.out.print("\t");
			}
			System.out.println("");
		}
	}
	
	public void valueIterationExperimenter(
		String outputPath,
		double[] gammaArray,
		int minIter,
		int stepSize,
		int maxIter,
		int numTrials,
		double viConvergenceCriterion
		) throws IOException{
		
		Path path = FileSystems.getDefault().getPath(outputPath);
		if (Files.notExists(path)) {
			new File(outputPath).mkdirs();
		}
			
		// setup csv
		CSVWriter writer = new CSVWriter(new FileWriter(outputPath + "vi_gamma_experiment.csv"));
		// write header
		writer.writeNext(new String[] {"numActions", "CumulativeReward", "PlanningTime (ms)", "meanV", "gamma", "maxIters", "trialNum"});

		List<Double[]> results = new ArrayList<Double[]>();
		DecimalFormat df = new DecimalFormat("#.####");
		for (double gamma : gammaArray) {
			System.out.println(gamma);
			
			// run experiment as a function of max iterations
			for (int trialNum=0; trialNum<numTrials; trialNum++){

				// convergence criteria
				double pastMeanV = Math.pow(10,10);
				int numConverged = 0;
				for (int numIters = minIter; numIters < maxIter; numIters += stepSize) {
				
					System.out.println("Starting run for gamma:" + gamma + " numIters:" + numIters + " trial:" + trialNum);
					Planner planner = new ValueIteration(domain, gamma, hashingFactory, viConvergenceCriterion / 10, numIters);
					
					Double[] experimentResultsOut = new Double[7];
					Double[] experimentResults = PlanningUtils.runEpisode(
						planner, domain, initialState, outputPath + "vi_gamma_" + df.format(gamma) + "_trial_" + trialNum
					);

					// get meanV convergence data
					List<State> allStates = StateReachability.getReachableStates(initialState, domain, hashingFactory);
									
					ValueFunction vF = (ValueFunction) planner;

					double sum = 0;
					for (State s : allStates) {
						sum += vF.value(s);
					}
					double meanV = sum / allStates.size();

					// prepare results
					experimentResultsOut[0] = experimentResults[0];
					experimentResultsOut[1] = experimentResults[1];
					experimentResultsOut[2] = experimentResults[2];
					experimentResultsOut[3] = meanV;
					experimentResultsOut[4] = gamma;
					experimentResultsOut[5] = (double) numIters;
					experimentResultsOut[6] = (double) trialNum;
					
					results.add(experimentResultsOut);
					String[] stringResults = new String[7];
					for (int recordIndex=0; recordIndex < experimentResultsOut.length; recordIndex++) {
						stringResults[recordIndex] = experimentResultsOut[recordIndex].toString();
					}
					
					writer.writeNext(stringResults);

					// check convergence
					System.out.println(meanV);
					if (Math.abs(meanV - pastMeanV) < viConvergenceCriterion) {
						System.out.println("Converged after " + numIters + " iterations");
						numConverged += 1;
					} else {
						numConverged = 0;
					}
					if (numConverged > 2) {
						break;
					}
					pastMeanV = meanV;
				}
			}
		}
		writer.close();
		for (Double[] result : results) {
			for (Double r : result ) {
				System.out.print(r);
				System.out.print("\t");
			}
			System.out.println("");
		}
	}

	public void policyIterationExperimenter(
		String outputPath,
		double[] gammaArray,
		int minIter,
		int stepSize,
		int maxIter,
		int numTrials,
		double piConvergenceCriterion
		) throws IOException{
		
		Path path = FileSystems.getDefault().getPath(outputPath);
		if (Files.notExists(path)) {
			new File(outputPath).mkdirs();
		}
			
		// setup csv
		CSVWriter writer = new CSVWriter(new FileWriter(outputPath + "pi_gamma_experiment.csv"));
		// write header
		writer.writeNext(new String[] {"numActions", "CumulativeReward", "PlanningTime (ms)", "meanV", "gamma", "maxIters", "trialNum"});

		List<Double[]> results = new ArrayList<Double[]>();
		DecimalFormat df = new DecimalFormat("#.####");
		for (double gamma : gammaArray) {
			System.out.println(gamma);
			
			// run experiment as a function of max iterations
			for (int trialNum=0; trialNum<numTrials; trialNum++){

				// convergence criteria
				double pastMeanV = Math.pow(10,10);
				int numConverged = 0;
				for (int numIters = minIter; numIters < maxIter; numIters += stepSize) {
				
					System.out.println("Starting run for gamma:" + gamma + " numIters:" + numIters + " trial:" + trialNum);
					Planner planner = new PolicyIteration(domain, gamma, hashingFactory, piConvergenceCriterion / 10, 10000, numIters);

					Double[] experimentResultsOut = new Double[7];
					Double[] experimentResults = PlanningUtils.runEpisode(
						planner, domain, initialState, outputPath + "pi_gamma_" + df.format(gamma) + "_trial_" + trialNum
					);

					// get meanV convergence data
					List<State> allStates = StateReachability.getReachableStates(initialState, domain, hashingFactory);
									
					ValueFunction vF = (ValueFunction) planner;

					double sum = 0;
					for (State s : allStates) {
						sum += vF.value(s);
					}
					double meanV = sum / allStates.size();

					// prepare results
					experimentResultsOut[0] = experimentResults[0];
					experimentResultsOut[1] = experimentResults[1];
					experimentResultsOut[2] = experimentResults[2];
					experimentResultsOut[3] = meanV;
					experimentResultsOut[4] = gamma;
					experimentResultsOut[5] = (double) numIters;
					experimentResultsOut[6] = (double) trialNum;
					
					results.add(experimentResultsOut);
					String[] stringResults = new String[7];
					for (int recordIndex=0; recordIndex < experimentResultsOut.length; recordIndex++) {
						stringResults[recordIndex] = experimentResultsOut[recordIndex].toString();
					}
					
					writer.writeNext(stringResults);

					// check convergence
					System.out.println(meanV);
					if (Math.abs(meanV - pastMeanV) < piConvergenceCriterion) {
						System.out.println("Converged after " + numIters + " iterations");
						numConverged += 1;
					} else {
						numConverged = 0;
					}
					if (numConverged > 2) {
						break;
					}
					pastMeanV = meanV;
				}
			}
		}
		writer.close();
		for (Double[] result : results) {
			for (Double r : result ) {
				System.out.print(r);
				System.out.print("\t");
			}
			System.out.println("");
		}
	}

	public static void main(String[] args) {
	
		BlockDudeProblem example = new BlockDudeProblem(0);
		String outputPath = "output_blockdude_test/"; //directory to record results

        //run example
        // example.AStarExample(outputPath);
        // example.BFSExample(outputPath);
        example.valueIterationExample(outputPath);

		//run the visualizer
		example.visualize(outputPath);		
	}
}