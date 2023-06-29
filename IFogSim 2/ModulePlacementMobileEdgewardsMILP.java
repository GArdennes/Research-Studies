package org.fog.placement;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Arrays;

import org.apache.commons.math3.util.Pair;
//import org.apache.commons.lang3.ArrayUtils;

import org.cloudbus.cloudsim.core.CloudSim;
import org.fog.application.AppEdge;
import org.fog.application.AppModule;
import org.fog.application.Application;
import org.fog.placement.ModuleMapping;
import org.fog.application.selectivity.SelectivityModel;
import org.fog.entities.Actuator;
import org.fog.entities.FogDevice;
import org.fog.entities.Sensor;
import org.fog.entities.Tuple;
import org.fog.utils.Logger;
import org.fog.utils.FogUtils;

import org.chocosolver.solver.*;
import org.chocosolver.solver.Model;
import org.chocosolver.solver.Solver;
import org.chocosolver.solver.variables.BoolVar;
import org.chocosolver.solver.variables.IntVar;
import org.chocosolver.util.tools.ArrayUtils;
import org.chocosolver.solver.search.*;
import org.chocosolver.solver.search.strategy.Search;
import org.chocosolver.solver.search.strategy.selectors.values.IntDomainMiddle;
import org.chocosolver.solver.search.strategy.selectors.variables.*;


public class ModulePlacementMobileEdgewardsMILP extends ModulePlacement{
	
	protected ModuleMapping moduleMapping;
	protected List<Sensor> sensors;
	protected List<Actuator> actuators;
	protected List<FogDevice> fogDevices;
	private Application application;
	protected Map<Integer, List<String>> ModuleMap;
	protected Model model;

	public ModulePlacementMobileEdgewardsMILP(List<FogDevice> fogDevices, List<Sensor> sensors, List<Actuator> actuators, 
			Application application, ModuleMapping moduleMapping){
		this.fogDevices = fogDevices;
		this.sensors = sensors;
		this.actuators = actuators;
		this.application = application;
		this.moduleMapping = moduleMapping;
		
		int Module_No = getApplication().getModules().size();
		int EdgeNode_No = fogDevices.size();
		int Bdw_C = 5;
		ModuleMap = new HashMap<>();
		int[] Node_Capacity = new int [Module_No];
		Arrays.fill(Node_Capacity, 5);
		int[][] Power = new int[EdgeNode_No][Module_No];
		
		for (int i = 0; i < EdgeNode_No; i++)
		{
			for (int j = 0; j < Module_No; j++)
			{
				Power[i][j] = (int) getApplication().getModules().get(j).getMips();
			}
		}
		
		model = new Model("ModulePlacement");
		
		OptimizationFunct(Module_No, EdgeNode_No, Power, Node_Capacity, Bdw_C);
		
		mapModules();
	}
	
	
	private void OptimizationFunct(int Module_No, int EdgeNode_No, int[][] Power, int[] Node_Capacity, int Bdw_C)
	{
		BoolVar[] placed = model.boolVarArray("placed", Module_No);
		IntVar[] execute = model.intVarArray("execute", EdgeNode_No, 1, Module_No, false);
		IntVar[] cost = model.intVarArray("cost", EdgeNode_No, 150, 450, true);
		IntVar total_cost = model.intVar("total_cost", 0, 1000000, true);
		
		for (int i = 0; i < EdgeNode_No; i++)
		{
			model.element(model.intVar(1), placed, execute[i], 1).post();
			model.element(cost[i], Power[i], execute[i], 1).post();
		}
		
		for (int i = 0; i < Module_No; i++)
		{
			IntVar occ = model.intVar("occur_" + i, 0, Node_Capacity[i], true);
			model.count(i + 1, execute, occ).post();
			occ.ge(placed[i]).post();
		}
		
		int[] coeffs = new int[Module_No + EdgeNode_No];
		Arrays.fill(coeffs, 0, Module_No, Bdw_C);
		Arrays.fill(coeffs, Module_No, Module_No + EdgeNode_No, 1);
		model.scalar(ArrayUtils.append(placed, cost), coeffs, "=", total_cost).post();
		
		model.setObjective(Model.MINIMIZE, total_cost);
		Solver solver = model.getSolver();
		solver.setSearch(Search.intVarSearch(
				new VariableSelectorWithTies<>(
						new FirstFail(model),
						new Smallest()
				),
				new IntDomainMiddle(false),
				ArrayUtils.append(execute, cost, placed)));
		solver.showShortStatistics();
		while(solver.solve())
		{
			currentMapModules(placed, Module_No, execute, EdgeNode_No);
		}	
	}
	
	protected void currentMapModules(IntVar[] placed, int Module_No, IntVar[] execute, int EdgeNode_No)
	{
		for (int i = 0; i < Module_No; i++)
		{
			if (placed[i].getValue() > 0)
			{
				for (int j = 0; j < EdgeNode_No; j++)
				{
					if (execute[j].getValue() == (i+1))
					{
						AppModule module = getApplication().getModules().get(i);
						FogDevice device = fogDevices.get(j);
						int deviceId = device.getId();
						
						List<String> moduleList = ModuleMap.getOrDefault(deviceId, new ArrayList<>());
						moduleList.add(module.getName());
						ModuleMap.put(deviceId, moduleList);
					}
				}
			}
		}
	}
	
	@Override
	protected void mapModules()
	{
		for(int deviceId : ModuleMap().keySet()){
			for(String module : ModuleMap().get(deviceId)){
				createModuleInstanceOnDevice(getApplication().getModuleByName(module), getFogDeviceById(deviceId));
			}
		}
	}


	private Map<Integer, List<String>> ModuleMap() 
	{
		// TODO Auto-generated method stub
		return ModuleMap;
	}
}
