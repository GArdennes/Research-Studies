package tech.skargen.recloud.developers.cypherskar.pso;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.cloudbus.cloudsim.Cloudlet;
import org.cloudbus.cloudsim.Vm;
import tech.skargen.recloud.components.cloudsim.IReBroker;
import tech.skargen.recloud.components.simulation.ASimulation;

public class MarkovDecisionProcess extends ASimulation
{
    private int numStates;
    private int numActions;
    private double[][] rewards;
    private double[][][] transitionProbabilities;
    private double discountFactor;
    private int maxIterations;
    private double[] stateValues;
    private int[] policy;
    private List<Vm> vms;
    private List<Cloudlet> cloudlets;

    public MarkovDecisionProcess(double discountFactor, int maxIterations) 
    {
		this.discountFactor = discountFactor;
        this.maxIterations = maxIterations; 
	}

    @Override
    public String getAlgorithmName()
    {
        return "Markov Decision Process";
    }

    @Override
    public String getDeveloper()
    {
        return "Kevin";
    }

    @Override
    public String getAdditionalInfo()
    {
        return null;
    }

    @Override
    public void startEntity(IReBroker rebroker){}

    @Override
    public void shutdownEntity(IReBroker rebroker){}

    @Override
    public void processCloudletsSubmit(IReBroker rebroker)
    {
        List<Cloudlet> cloudlets = rebroker.getEntity().getCloudletList();
        List<Vm> vms = rebroker.getEntity().getVmsCreatedList();

        rebroker.updateProgressMax(maxIterations);
        rebroker.updateProgressMessage("Running Markov Decision Process");

        findSolution(cloudlets, vms, rebroker);
    }

    @Override
    public <T extends Cloudlet> void processCloudletReturn(IReBroker rebroker, T Task){}

    public void findSolution(final List<Cloudlet> tasks, final List<Vm> vms, IReBroker rebroker)
    {
       numStates = tasks.size();
       numActions = vms.size();
       rewards = new double[numStates][numActions];
       transitionProbabilities = new double[numStates][numActions][numStates];

       Comparator<Cloudlet> comp = new Comparator<Cloudlet>()
       {
           public int compare(Cloudlet a, Cloudlet b)
           {
               return (int) ((b.getCloudletLength() * b.getNumberOfPes()) - (a.getCloudletLength() * a.getNumberOfPes()));
           }
       };

       tasks.sort(comp);

       for (int state = 0; state < numStates; state++)
       {
           Cloudlet task = tasks.get(state);

           for (int action = 0; action < numActions; action++)
           {
               Vm vm = vms.get(action);
               if (this.ExecutionTime(task,vm) < 5)
               {
                   rewards[state][action] = 10.0;
               } else if (this.ExecutionTime(task, vm) < 50)
               {
                   rewards[state][action] = 5.0;
               } else if (this.ExecutionTime(task, vm) < 100)
               {
                   rewards[state][action] = 2.0;
               } else {
                   rewards[state][action] = 1.0;
               }

               for (int nextState = 0; nextState < numStates; nextState++)
               {
                   transitionProbabilities[state][action][nextState] = 5.0;
               }
           }
       }

       run();

       int[] policy = getOptimalPolicy();
       for (int i = 0; i < numStates; i++)
       {
           Cloudlet task = tasks.get(i);
           Vm vm = vms.get(policy[i]);
           task.setVmId(vm.getId());
           rebroker.submitCloudlet(task);
           rebroker.updateProgress(1);
       }
    }

    public double ExecutionTime(Cloudlet task, Vm vm)
    {
        if (vm.getHost() == null)
            return task.getCloudletLength() / vm.getMips();
        return task.getCloudletLength() / vm.getHost().getTotalAllocatedMipsForVm(vm);
    }

    public void run()
    {
        policy = new int [numStates];
        stateValues = new double[numStates];
        
        //Perform value iteration
        for (int iteration = 0; iteration < maxIterations; iteration++)
        {
            double[] newValues = new double[numStates];
            int[] newPolicy = new int[numStates];

            for (int state = 0; state < numStates; state++)
            {
                double maxValue = Double.NEGATIVE_INFINITY;
                int bestAction = 0;

                for (int action = 0; action < numActions; action++)
                {
                    double actionValue = calculateActionValue(state, action);
                    if (actionValue > maxValue)
                    {
                        maxValue = actionValue;
                        bestAction = action;
                    }
                }

                newValues[state] = maxValue;
                newPolicy[state] = bestAction;
            }

            stateValues = newValues;
            policy = newPolicy;
        }
    }

    private double calculateActionValue(int state, int action)
    {
        double actionValue = 0.0;

        for (int nextState = 0; nextState < numStates; nextState++)
        {
            double transitionProbability = transitionProbabilities[state][action][nextState];
            double reward = rewards[state][action];
            double discountedValue = discountFactor * stateValues[nextState];
            actionValue += transitionProbability * (reward + discountedValue);
        }

        return actionValue;
    }

    public int[] getOptimalPolicy()
    {
        return policy;
    }
}