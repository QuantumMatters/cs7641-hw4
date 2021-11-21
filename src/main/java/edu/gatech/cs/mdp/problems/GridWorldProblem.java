package edu.gatech.cs.mdp.problems;

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
import burlap.behavior.singleagent.planning.deterministic.informed.Heuristic;
import burlap.behavior.singleagent.planning.deterministic.informed.astar.AStar;
import burlap.behavior.singleagent.planning.deterministic.uninformed.bfs.BFS;
import burlap.behavior.singleagent.planning.deterministic.uninformed.dfs.DFS;
import burlap.behavior.singleagent.planning.stochastic.policyiteration.PolicyIteration;
import burlap.behavior.singleagent.planning.stochastic.valueiteration.ValueIteration;
import burlap.behavior.valuefunction.QProvider;
import burlap.behavior.valuefunction.ValueFunction;
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
import burlap.mdp.singleagent.common.GoalBasedRF;
import burlap.mdp.singleagent.common.VisualActionObserver;
import burlap.mdp.singleagent.environment.SimulatedEnvironment;
import burlap.mdp.singleagent.model.FactoredModel;
import burlap.mdp.singleagent.model.RewardFunction;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.statehashing.HashableStateFactory;
import burlap.statehashing.simple.SimpleHashableStateFactory;
import burlap.visualizer.Visualizer;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.List;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;

import com.opencsv.CSVWriter;

import java.util.ArrayList;

import edu.gatech.cs.mdp.utils.Maze;
import edu.gatech.cs.mdp.utils.PlanningUtils;


public class GridWorldProblem {

	
	
	GridWorldDomain gwdg;
	OOSADomain domain;
	RewardFunction rf;
	TerminalFunction tf;
	StateConditionTest goalCondition;
	State initialState;
	HashableStateFactory hashingFactory;
	SimulatedEnvironment env;
	int width;
	int height;
	

	public GridWorldProblem(
		String MazeFile,
		double transProb,
		double goalReward,
		double hazardReward,
		double normalTileReward

		){
		
		// read a csv representing the grid world
		Maze maze  = new Maze(MazeFile);
		gwdg = new GridWorldDomain(maze.getHeight(), maze.getWidth());
		gwdg.setMap(maze.getMap());
		gwdg.setProbSucceedTransitionDynamics(transProb);

		this.width = maze.getWidth();
		this.height = maze.getHeight();

		// handle goal
		int[] goal = maze.getGoal();
		tf = new GridWorldTerminalFunction(goal[0], goal[1]);
		gwdg.setTf(tf);
		goalCondition = new TFGoalCondition(tf);

		// handle start
		int[] start = maze.getStart();
		initialState = new GridWorldState(new GridAgent(start[0], start[1]),
										  new GridLocation(goal[0], goal[1], "loc0"));

		// handle rewards
		GridWorldRewardFunction rf = new GridWorldRewardFunction(this.width, this.height, normalTileReward);

		rf.setReward(goal[0], goal[1], goalReward);
		for (int[] hazard : maze.getHazards()) {
			rf.setReward(hazard[0], hazard[1], hazardReward);
		}
		gwdg.setRf(rf);


		domain = gwdg.generateDomain();
		hashingFactory = new SimpleHashableStateFactory();

		env = new SimulatedEnvironment(domain, initialState);

	}


	public void visualize(String outputPath){
		Visualizer v = GridWorldVisualizer.getVisualizer(gwdg.getMap());
		new EpisodeSequenceVisualizer(v, domain, outputPath);
	}


	public void BFSExample(String outputPath){
		
		DeterministicPlanner planner = new BFS(domain, goalCondition, hashingFactory);
		Policy p = planner.planFromState(initialState);
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "bfs");
			
	}


	public void DFSExample(String outputPath){
		
		DeterministicPlanner planner = new DFS(domain, goalCondition, hashingFactory);
		Policy p = planner.planFromState(initialState);
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "dfs");
		
	}


	public void AStarExample(String outputPath){
		
		Heuristic mdistHeuristic = new Heuristic() {
	
			public double h(State s) {
				GridAgent a = ((GridWorldState)s).agent;
				double mdist = Math.abs(a.x-10) + Math.abs(a.y-10);
	
				return -mdist;
			}
		};
	
		DeterministicPlanner planner = new AStar(domain, goalCondition, hashingFactory,
												 mdistHeuristic);
	
		Policy p = planner.planFromState(initialState);
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "astar");
		
	}		
	
	
	public void simpleValueFunctionVis(ValueFunction valueFunction, Policy p){
		
		List<State> allStates = StateReachability.getReachableStates(initialState, 
		domain, hashingFactory);
		ValueFunctionVisualizerGUI gui = GridWorldDomain.getGridWorldValueFunctionVisualization(
			allStates, this.width, this.height, valueFunction, p);
			gui.initGUI();
			
	}


	public void valueIterationExample(String outputPath){
		
		Planner planner = new ValueIteration(domain, 0.99, hashingFactory, 0.001, 10000);
		Policy p = planner.planFromState(initialState);
	
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "vi");

		simpleValueFunctionVis((ValueFunction)planner, p);
		
	}


	public void policyIterationExample(String outputPath){
		
		Planner planner = new PolicyIteration(domain, 0.99, hashingFactory, 0.001, 100, 100);
		Policy p = planner.planFromState(initialState);
	
		PolicyUtils.rollout(p, initialState, domain.getModel()).write(outputPath + "pi");

		simpleValueFunctionVis((ValueFunction)planner, p);
		
	}



	public void QLearningExample(String outputPath){
		
		LearningAgent agent = new QLearning(domain, 0.99, hashingFactory, 0.3, 0.99);
	
		//run learning for 50 episodes
		for(int i = 0; i < 200; i++){
			Episode e = agent.runLearningEpisode(env);
	
			e.write(outputPath + "ql_" + i);
			System.out.println(i + ": " + e.maxTimeStep());
	
			//reset environment for next learning episode
			env.resetEnvironment();
		}
		
	}


	public void SarsaLearningExample(String outputPath){
		
		LearningAgent agent = new SarsaLam(domain, 0.99, hashingFactory, 0., 0.5, 0.3);
	
		//run learning for 50 episodes
		for(int i = 0; i < 50; i++){
			Episode e = agent.runLearningEpisode(env);
	
			e.write(outputPath + "sarsa_" + i);
			System.out.println(i + ": " + e.maxTimeStep());
	
			//reset environment for next learning episode
			env.resetEnvironment();
		}
		
	}

	public void experimenterAndPlotter(){
		
		//different reward function for more structured performance plots
		((FactoredModel)domain.getModel()).setRf(new GoalBasedRF(this.goalCondition, 5.0, -0.1));
		
		/**
		 * Create factories for Q-learning agent and SARSA agent to compare
		 */
		LearningAgentFactory qLearningFactory = new LearningAgentFactory() {

			public String getAgentName() {
				return "Q-Learning";
			}


			public LearningAgent generateAgent() {
				return new QLearning(domain, 0.99, hashingFactory, 0.3, 0.1);
			}
		};

		LearningAgentFactory sarsaLearningFactory = new LearningAgentFactory() {

			public String getAgentName() {
				return "SARSA";
			}


			public LearningAgent generateAgent() {
				return new SarsaLam(domain, 0.99, hashingFactory, 0.0, 0.1, 1.);
			}
		};

		LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter(
			env, 10, 100, qLearningFactory, sarsaLearningFactory);
		exp.setUpPlottingConfiguration(500, 250, 2, 1000,
				TrialMode.MOST_RECENT_AND_AVERAGE,
				PerformanceMetric.CUMULATIVE_STEPS_PER_EPISODE,
				PerformanceMetric.AVERAGE_EPISODE_REWARD);

		exp.startExperiment();
		exp.writeStepAndEpisodeDataToCSV("expData");
	
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
					
					Double[] experimentResultsOut = new Double[5];
					Double experimentResults = PlanningUtils.runPlanning(
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
					experimentResultsOut[0] = experimentResults;
					experimentResultsOut[1] = meanV;
					experimentResultsOut[2] = gamma;
					experimentResultsOut[3] = (double) numIters;
					experimentResultsOut[4] = (double) trialNum;
					
					results.add(experimentResultsOut);
					String[] stringResults = new String[5];
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

	public void policyIterationMeanV(
		String outputPath,
		double[] gammaArray,
		int minIter,
		int stepSize,
		int maxIter,
		int numTrials,
		double piConvergenceCriterion
		) throws IOException {
		
		Path path = FileSystems.getDefault().getPath(outputPath);
		if (Files.notExists(path)) {
			new File(outputPath).mkdirs();
		}

		// setup csv
		CSVWriter writer = new CSVWriter(new FileWriter(outputPath + "pi_gamma_experiment_just_meanV.csv"));
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
				for (int numIters = minIter; numIters < maxIter; numIters += stepSize) {
				
					System.out.println("Starting run for gamma:" + gamma + " numIters:" + numIters + " trial:" + trialNum);
					Planner planner = new PolicyIteration(domain, gamma, hashingFactory, piConvergenceCriterion / 10, 10000, numIters);

					Double[] experimentResultsOut = new Double[5];
					Double experimentResults = PlanningUtils.runPlanning(
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
					experimentResultsOut[0] = experimentResults;
					experimentResultsOut[1] = meanV;
					experimentResultsOut[2] = gamma;
					experimentResultsOut[3] = (double) numIters;
					experimentResultsOut[4] = (double) trialNum;
					
					results.add(experimentResultsOut);
					String[] stringResults = new String[5];
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

	public void qLearningRateExperimenter(){
		
		//different reward function for more structured performance plots
		// ((FactoredModel)domain.getModel()).setRf(new GoalBasedRF(this.goalCondition, 5.0, -0.1));
		
		/**
		 * Create factories for Q-learning agent and SARSA agent to compare
		 */
		List<LearningAgentFactory> factories = new ArrayList<LearningAgentFactory>();
		for (int i : new int[] {70, 75, 99}) {
			double learningRate = (double) i / 100;
			LearningAgentFactory qLearningFactory = new LearningAgentFactory() {

				public String getAgentName() {
					return "Q-Learning LR: " + learningRate;
				}


				public LearningAgent generateAgent() {
					return new QLearning(domain, 0.99, hashingFactory, 0.3, learningRate);
				}
			};
			factories.add(qLearningFactory);
		}
		LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter(
			env, 50, 20, factories.toArray(new LearningAgentFactory[0]));
		exp.setUpPlottingConfiguration(500, 250, 2, 1000,
				TrialMode.MOST_RECENT_AND_AVERAGE,
				PerformanceMetric.CUMULATIVE_STEPS_PER_EPISODE,
				PerformanceMetric.AVERAGE_EPISODE_REWARD);

		exp.startExperiment();
		exp.writeStepAndEpisodeDataToCSV("qLearningRateExpData");
	
	}

	public void qLearningGammaExperimenter(){

		List<LearningAgentFactory> factories = new ArrayList<LearningAgentFactory>();
		for (int i : new int[] {90, 95, 99}) {
			double gamma = (double) i / 100;
			LearningAgentFactory qLearningFactory = new LearningAgentFactory() {

				public String getAgentName() {
					return "Q-Learning Gamma: " + gamma;
				}


				public LearningAgent generateAgent() {
					return new QLearning(domain, gamma, hashingFactory, 0.3, 0.9);
				}
			};
			factories.add(qLearningFactory);
		}
		LearningAlgorithmExperimenter exp = new LearningAlgorithmExperimenter(
			env, 20, 20, factories.toArray(new LearningAgentFactory[0]));
		exp.setUpPlottingConfiguration(500, 250, 2, 1000,
				TrialMode.MOST_RECENT_AND_AVERAGE,
				PerformanceMetric.CUMULATIVE_STEPS_PER_EPISODE,
				PerformanceMetric.AVERAGE_EPISODE_REWARD,
				PerformanceMetric.STEPS_PER_EPISODE);

		exp.startExperiment();
		exp.writeStepAndEpisodeDataToCSV("qLearningGammaExpData");
	
	}

	public static void main(String[] args) {
	
		// GridWorldProblem example = new GridWorldProblem("maze6.csv");
		// String outputPath = "output_maze6/"; //directory to record results
		// 
		// //run example
		// //example.BFSExample(outputPath);
		// // example.QLearningExample(outputPath);
		// // example.valueIterationExample(outputPath);
		// // example.policyIterationExample(outputPath);
		// //example.experimenterAndPlotter();
		// //example.qLearningRateExperimenter();
		// //example.qLearningGammaExperimenter();
		// 
		// try {
		// 	example.valueIterationExperimenter(outputPath);
		// } catch (IOException e) {
		// 	// TODO Auto-generated catch block
		// 	e.printStackTrace();
		// }
// 
		// //run the visualizer
		// example.visualize(outputPath);		
	}
}