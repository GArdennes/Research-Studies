package org.fog.placement;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Comparator;
//import org.apache.commons.lang3.ArrayUtils;

import org.fog.application.AppModule;
import org.fog.application.Application;
import org.fog.entities.Actuator;
import org.fog.entities.FogDevice;
import org.fog.entities.Sensor;



public class ModulePlacementMobileEdgewardsMDP extends ModulePlacement
{
    private int numStates;
    private int numActions;
    private double[][] rewards;
    private double[][][] transitionProbabilities;
    private double discountFactor;
    private int maxIterations;
    private double [] stateValues;
    private int[] policy;

    protected ModuleMapping moduleMapping;
    protected List<Sensor> sensors;
    protected List<Actuator> actuators;
    protected List<FogDevice> fogDevices;
	private Application application;
	protected Map<Integer, List<String>> moduleMap;


    public ModulePlacementMobileEdgewardsMDP(List<FogDevice> fogDevices, List<Sensor> sensors, List<Actuator> actuators, Application application, ModuleMapping moduleMapping) 
    {
        this.discountFactor = 0.9;
        this.maxIterations = 100;
        moduleMap = new HashMap<>();

        this.fogDevices = fogDevices;
        this.sensors = sensors;
        this.actuators = actuators;
        this.application = application;
        this.moduleMapping = moduleMapping;
        
        
        findSolution(application, fogDevices);
        mapModules();
	}

    public void findSolution(Application application, List<FogDevice> fogDevices)
    {
        numStates = application.getModules().size();
        numActions = fogDevices.size();
        rewards = new double[numStates][numActions];
        transitionProbabilities = new double[numStates][numActions][numStates];

        List<AppModule> modules = new ArrayList<>(numStates);
        for (AppModule module : application.getModules())
        {
            modules.add(module);
        }

        Comparator<AppModule> comp = new Comparator<AppModule>() 
        {
            @Override
            public int compare(AppModule a, AppModule b) {
                return Double.compare(b.getMips() * b.getNumInstances(), a.getMips() * a.getNumInstances());
            }
        };


        modules.sort(comp);

        for (int state = 0; state < numStates; state++)
        {
            AppModule module = modules.get(state);
            for (int action = 0; action < numActions; action++)
            {
                FogDevice device = fogDevices.get(action);
                if (this.ExecutionTime(module, device) < 5)
                {
                    rewards[state][action] = 10.0;
                } else if (this.ExecutionTime(module, device) < 50)
                {
                    rewards[state][action] = 5.0;
                } else if (this.ExecutionTime(module, device) < 100)
                {
                    rewards[state][action] = 2.0;
                } else {
                    rewards[state][action] = 1.0;
                }

                for (int nextState = 0; nextState < numStates; nextState++)
                {
                    transitionProbabilities[state][action][nextState] = 5.0;
                    transitionProbabilities[0][action][1] = 10.0;
                    transitionProbabilities[1][action][2] = 10.0;
                    transitionProbabilities[2][action][3] = 10.0;
                    transitionProbabilities[2][action][4] = 10.0;
                    transitionProbabilities[3][action][0] = 10.0;
                    transitionProbabilities[4][action][0] = 10.0;
                }
            }
        }

        run();
    }

    public double ExecutionTime(AppModule module, FogDevice device)
    {
        if (device.getHost() == null)
            return module.getMips() * device.getRatePerMips();
        return module.getMips() / device.getHost().getTotalMips();
    }

    public void run()
    {
        policy = new int [numStates];
        stateValues = new double[numStates];

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

        currentMapModules();
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

    protected void currentMapModules()
    {
        int[] policy = getOptimalPolicy();
        
        for (int i = 0; i < numStates; i++)
        {
            for (int j = 0; j < numActions; j++)
            {
                AppModule module = getApplication().getModules().get(i);
                FogDevice device = fogDevices.get(policy[i]);
                int deviceId = device.getId();

                List<String> moduleList = moduleMap().getOrDefault(deviceId, new ArrayList<String>());
                moduleList.add(module.getName());
                moduleMap().put(deviceId, moduleList);
            }
        }
    }

    @Override
    public void mapModules()
    {
        for (int deviceId : moduleMap().keySet())
        {
            for(String module : moduleMap().get(deviceId))
            {
                createModuleInstanceOnDevice(getApplication().getModuleByName(module), getFogDeviceById(deviceId));
            }
        }
    }

    public int[] getOptimalPolicy()
    {
        return policy;
    }

    private Map<Integer, List<String>> moduleMap()
    {
        return moduleMap;
    }
}