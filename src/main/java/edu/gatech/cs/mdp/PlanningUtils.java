package edu.gatech.cs.mdp;

import burlap.behavior.singleagent.planning.Planner;
import burlap.behavior.valuefunction.ValueFunction;
import burlap.behavior.singleagent.Episode;
import burlap.mdp.core.state.State;
import burlap.mdp.singleagent.oo.OOSADomain;
import burlap.behavior.policy.Policy;
import burlap.behavior.policy.PolicyUtils;

import java.lang.Math;
import java.util.List;
import java.util.ArrayList;

import edu.gatech.cs.mdp.GridWorldProblem;

public class PlanningUtils {

    /* Runs an episode of a planning algorithm collecting the total number of actions, the cumulative reward,
    and the planning time in milliseconds.
    */
    public static Double[] runEpisode(Planner planner, OOSADomain domain, State initialState, String outputPath) {

        long startTime = System.nanoTime();
        Policy p = planner.planFromState(initialState);
        long endTime = System.nanoTime();
        Double planningTimeMillis = (double) (endTime - startTime) * Math.pow(10, -6) ;

        Episode e = PolicyUtils.rollout(p, initialState, domain.getModel());
        
        double totalReward = 0;
        for (double r : e.rewardSequence) {
            totalReward += r;
        }

        e.write(outputPath);
        // simpleValueFunctionVis((ValueFunction)planner, p);

        // format output
        Double[] output = new Double[3];
        output[0] = ((double) e.numActions());
        output[1] = (totalReward);
        output[2] = (planningTimeMillis);

        return output;
    }
}
