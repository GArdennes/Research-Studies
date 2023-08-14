package org.fog.test.perfeval;

import org.cloudbus.cloudsim.Host;
import org.cloudbus.cloudsim.Log;
import org.cloudbus.cloudsim.Pe;
import org.cloudbus.cloudsim.Storage;
import org.cloudbus.cloudsim.core.CloudSim;
import org.cloudbus.cloudsim.power.PowerHost;
import org.cloudbus.cloudsim.provisioners.RamProvisionerSimple;
import org.cloudbus.cloudsim.sdn.overbooking.BwProvisionerOverbooking;
import org.cloudbus.cloudsim.sdn.overbooking.PeProvisionerOverbooking;
import org.fog.application.AppEdge;
import org.fog.application.AppLoop;
import org.fog.application.Application;
import org.fog.application.selectivity.FractionalSelectivity;
import org.fog.entities.*;
import org.fog.mobilitydata.DataParser;
import org.fog.mobilitydata.RandomMobilityGenerator;
import org.fog.mobilitydata.References;
import org.fog.placement.*;
import org.fog.policy.AppModuleAllocationPolicy;
import org.fog.scheduler.StreamOperatorScheduler;
import org.fog.utils.Config;
import org.fog.utils.FogLinearPowerModel;
import org.fog.utils.FogUtils;
import org.fog.utils.TimeKeeper;
import org.fog.utils.distribution.DeterministicDistribution;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.*;

public class DroneCoordinationSimulation
{
    //list of abstracted objects
    static List<FogDevice> fogDevices = new ArrayList<FogDevice>();
    static List<Sensor> sensors = new ArrayList<Sensor>();
    static List<Actuator> actuators = new ArrayList<Actuator> ();
    static Map<Integer, Integer> userMobilityPattern = new HashMap<Integer, Integer>();
    static LocationHandler locator;

    static boolean CLOUD = false;
    
    static double SENSOR_TRANSMISSION_TIME = 10; //decided based on ...
    static int numberOfMobileUsers = 1;

    //random mobility generator; 
    //1.creates random mobility dataset
    //2.refreshes dataset
    //3.initializes organizational structure to enhance data processing (clustering fog lvl)
    static boolean randomMobility_generator = true; 
    static boolean renewDataset = false;
    static List<Integer> clusteringLevels = new ArrayList<Integer>();

    public static void main(String[] args) {
	//print message indicating the application is starting
    Log.printLine("Starting Drone Navigation application...");

    try 
    {
        //disable logging messages
        Log.disable();

        //Initialize Simulator with 
        //1.number of users,
        //2. current time, 
        //3. trace flag(a config setting used to alter behaviour of devices)
        int num_user = 1;
        Calendar calendar = Calendar.getInstance();
        boolean trace_flag = false;
        CloudSim.init(num_user, calendar, trace_flag);

        //set the application ID and create a broker to manage the application
        //brokers act as intermediary between the user and the Fog devices
        String appId = "DroneCoordinationSimulation";
        FogBroker broker = new FogBroker("broker");

        //Create the application and associate it with the broker
        Application application = createApplication(appId, broker.getId());
        application.setUserId(broker.getId());

        //Initialize a data parser and LocationHandler objects
        //data parser converts from one format to another
        DataParser dataObject = new DataParser();
        locator = new LocationHandler(dataObject);

        //set the dataset reference depending on a boolean value
        String datasetReference = References.dataset_reference;
        if (randomMobility_generator)
        {
            datasetReference = References.dataset_random;
            createRandomMobilityDatasets(References.random_walk_mobility_model, datasetReference, renewDataset);
        }

        //create mobile users and fog devices for the application
        createMobileUser(broker.getId(), appId, datasetReference);
        createFogDevices(broker.getId(), appId);

        //create a module mapping to associate application modules with devices
        ModuleMapping moduleMapping = ModuleMapping.createModuleMapping();
        
        //Create a mobility controller to manage mobile user movement and module placement
        MobilityController controller = new MobilityController("master-controller", fogDevices, sensors, actuators, locator);

        //submit the application to the controller
        controller.submitApplication(application, 0, (new ModulePlacementMobileEdgewards(fogDevices, sensors, actuators, application, moduleMapping)));

        //set the simulation start time and start the simulation
        TimeKeeper.getInstance().setSimulationStartTime(Calendar.getInstance().getTimeInMillis());
        CloudSim.startSimulation();

        //stop the simulation and print a message indicating the application has finished
        CloudSim.stopSimulation();
        Log.printLine("Program is finished");
    }	catch (Exception e)
    {
        //catch any exceptions that occur and print a stack trace
        e.printStackTrace();
        Log.printLine("Error please try again later");
    }
	}

    /**
    * Creates random mobility datasets using a Random Mobility generator
    * @param mobilityModel
    * @param datasetReference
    * @param renewDataset
    * @throws IOException
    * @throws ParseException
    */
    private static void createRandomMobilityDatasets(int mobilityModel, String datasetReference, boolean renewDataset) throws IOException, ParseException 
    {
        RandomMobilityGenerator randMobilityGenerator = new RandomMobilityGenerator();
        for (int i = 0; i < numberOfMobileUsers; i++)
        {
            randMobilityGenerator.createRandomData(mobilityModel, i + 1, datasetReference, renewDataset);
        }
    }

    /**
    * Creates mobile users and adds them to the physical topology.
    * by looping through the number of mobile users and creating a new mobile device for each user.
    * @param userId
    * @param appId
    * @param datasetReference
    * @throws IOException
    */
    private static void createMobileUser(int userId, String appId, String datasetReference) throws IOException
    {
        for (int id = 1; id <= numberOfMobileUsers; id++)
            userMobilityPattern.put(id, References.DIRECTIONAL_MOBILITY);
        
        locator.parseUserInfo(userMobilityPattern, datasetReference);

        List<String> mobileUserDataIds = locator.getMobileUserDataId();

        for (int i = 0; i < numberOfMobileUsers; i++)
        {
            //setting up mobile device as Fog Device
            FogDevice mobile = addMobile("mobile_" + i, userId, appId, References.NOT_SET);
            //latency of connection between the smartphone and server is 2ms determined by ...
            mobile.setUplinkLatency(2);
            locator.linkDataWithInstance(mobile.getId(), mobileUserDataIds.get(i));
            mobile.setLevel(3);

            fogDevices.add(mobile);
        }
    }

    /**
    * Creates the fog devices in the physical topology of the simulation
    * Fog devices take 
    * 1. node name
    * 2. Million Instructions Per Second (mips)
    * 3. RAM 
    * 4. Uplink bandwidth
    * 5. Downlink bandwidth
    * 6. Cost of computing (ratePerMips)
    * 7. busy power rating
    * 8. idle power rating
    * @param userId
    * @param appId
    * @throws IOException
    * @throws NumberFormatException
    */
    private static void createFogDevices(int userId, String appId) throws NumberFormatException, IOException 
    {
        locator.parseResourceInfo();

        if (locator.getLevelWiseResources(locator.getLevelID("Cloud")).size() == 1) 
        {
            
            FogDevice cloud = createFogDevice("cloud", 44800, 40000, 100, 10000, 0.01, 16 * 103, 16 * 83.25); //determined by ...
            cloud.setParentId(References.NOT_SET);
            locator.linkDataWithInstance(cloud.getId(), locator.getLevelWiseResources(locator.getLevelID("Cloud")).get(0));
            cloud.setLevel(0);
            fogDevices.add(cloud);

            for (int i = 0; i < locator.getLevelWiseResources(locator.getLevelID("Proxy")).size(); i++)
            {
                FogDevice proxy = createFogDevice("proxy-server_" + i, 2800, 4000, 10000, 10000, 0.0, 107.339, 83.4333); //determined by ...
                locator.linkDataWithInstance(proxy.getId(), locator.getLevelWiseResources(locator.getLevelID("Proxy")).get(i));
                proxy.setParentId(cloud.getId());
                //latency of connection from proxy server to the cloud is 100ms determined by ...
                proxy.setUplinkLatency(100);
                proxy.setLevel(1);
                fogDevices.add(proxy);
            }

            for (int i = 0; i < locator.getLevelWiseResources(locator.getLevelID("Gateway")).size(); i++)
            {
                FogDevice gateway = createFogDevice("gateway_"+i, 2800, 4000, 10000, 10000, 0.0, 107.339, 83.4333); //determined by ...
                locator.linkDataWithInstance(gateway.getId(), locator.getLevelWiseResources(locator.getLevelID("Gateway")).get(i));
                gateway.setParentId(locator.determineParent(gateway.getId(), References.SETUP_TIME));
                gateway.setUplinkLatency(4);
                gateway.setLevel(2);
                fogDevices.add(gateway);
            }
        }
    }

    /**
    * Creates the peripherals for the mobile device and adds them to the simulation
    * @param name
    * @param userId
    * @param appId
    * @param parentId
    * @return mobile
    */
    private static FogDevice addMobile(String name, int userId, String appId, int parentId)
    {
        FogDevice mobile = createFogDevice(name, 200, 2048, 10000, 270, 0, 87.53, 82.44);
        mobile.setParentId(parentId);
        Sensor mobileSensor = new Sensor("sensor-"+name, "MOTION SENSOR", userId, appId, new DeterministicDistribution(SENSOR_TRANSMISSION_TIME));
        sensors.add(mobileSensor);
        Actuator mobileDisplay = new Actuator("actuator-"+name, userId, appId, "DISPLAY");
        //Actuator mobileMotion = new Actuator("actuator-"+name, userId, appId, "MOTION");
        actuators.add(mobileDisplay);
        //actuators.add(mobileMotion);
        mobileSensor.setGatewayDeviceId(mobile.getId());
        //latency of connection between sensor and mobile device is determined by ...
        mobileSensor.setLatency(6.0);
        mobileDisplay.setGatewayDeviceId(mobile.getId());
        //latency of connection between display and mobile device is determined by ...
        mobileDisplay.setLatency(1.0);
        //mobileMotion.setGatewayDeviceId(mobile.getId());
        //latency of connection between Motion and mobile device is determined by ...
        //mobileMotion.setLatency(1);
        return mobile;
    }

    /**
    * Creates a vanilla fog device (a microcloud device between the edge devices and the cloud)
    * @param node name
    * @param mips
    * @param RAM
    * @param upBw
    * @param downBw
    * @param level
    * @param ratePerMips
    * @param busyPower
    * @param idlePower
    * @return fogdevice
    */
    private static FogDevice createFogDevice(String nodeName, long mips, int ram, long upBw, long downBw, double ratePerMips, double busyPower, double idlePower)
    {
        List<Pe> peList = new ArrayList<Pe>();

        //3. Create PEs (Processing Element) and add these into a list
        peList.add(new Pe(0, new PeProvisionerOverbooking(mips)));

        int hostId = FogUtils.generateEntityId();
        long storage = 1000000;
        int bw = 10000;

        PowerHost host = new PowerHost (
            hostId,
            new RamProvisionerSimple(ram),
            new BwProvisionerOverbooking(bw),
            storage,
            peList,
            new StreamOperatorScheduler(peList),
            new FogLinearPowerModel(busyPower, idlePower)
        );

        List<Host> hostList = new ArrayList<Host>();
        hostList.add(host);

        String arch = "x86"; //system architecture
        String os = "Linux"; //operating system
        String vmm = "Xen";
        double time_zone = 2.0; //time zone this resource is located GMT+[time_zone]
        double cost = 3.0; //cost of using processing in this resource
        double costPerMem = 0.05; //the cost of using memory in this resource
        double costPerStorage = 0.001; //the cost of using storage in this resource
        double costPerBw = 0.05; //the cost of using bw in this resource
        LinkedList<Storage> storageList = new LinkedList<Storage>();

        //This instance of FogDevice specifies...
        //1. name
        //2. FogDeviceCharacteristics
        //3. VmAllocationPolicy
        //4. storageList
        //5. Scheduling interval
        //6. Uplink bw
        //7. Downlink bw
        //8. Uplink latency
        //9. ratePerMips
        FogDeviceCharacteristics characteristics = new FogDeviceCharacteristics (
            arch, os, vmm, host, time_zone, cost, costPerMem, costPerStorage, costPerBw
        );

        FogDevice fogdevice = null;
        try {
            fogdevice = new FogDevice(
                nodeName, characteristics, new AppModuleAllocationPolicy(hostList), storageList, 10, upBw, downBw, 0, ratePerMips
            );
        } catch (Exception e) {
            e.printStackTrace();
        }

        return fogdevice;
    }

    /**
    * Function to create application for the mobile user
    * @param appId
    * @param userId
    * @return
    */
    @SuppressWarnings({"serial"})
    private static Application createApplication(String appId, int userId)
    {
        Application application = Application.createApplication(appId, userId);

        Random random = new Random();

        //define range for each parameter
        int minRam = 11;
        int maxRam = 45;
        int minMips = 12;
        int maxMips = 21;
        int minSize = 10;
        int maxSize = 31;
        
        //add modules with random parameters to the application according to the directed acyclic graph
        application.addAppModule("client", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("communication microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("navigation microservice", minRam + random.nextInt(maxRam - minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("telemetry microservice", minRam + random.nextInt(maxRam - minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("mission planning microservice",minRam + random.nextInt(maxRam - minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("collision avoidance microservice",minRam + random.nextInt(maxRam - minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("battery management microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("failover microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("weather analysis microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("authentication microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("analytics microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("data synchronization microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("health monitoring microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("localization microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("resource allocation microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("traffic control microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("image processing microservice",minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("sensor integration microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("logistics and supply chain microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("reporting and monitoring microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("ai inference microservice", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        application.addAppModule("user interface", minRam + random.nextInt(maxRam-minRam), minMips + random.nextInt(maxMips - minMips), minSize + random.nextInt(maxSize - minSize));
        
        double minCpuLength = 1000;
        double maxCpuLength = 4000;
        
        //connecting the application modules in the application in the going direction and then the coming direction according to the graph
        application.addAppEdge("MOTION SENSOR", "client", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "MOTION SENSOR", Tuple.UP, AppEdge.SENSOR);
        application.addAppEdge("client", "communication microservice", 1500, 500, "DATA", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("communication microservice", "navigation microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("navigation microservice", "telemetry microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "MOTION_CONTROL", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("telemetry microservice", "mission planning microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA2", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("mission planning microservice", "collision avoidance microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA3", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("collision avoidance microservice", "battery management microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA4", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("battery management microservice", "failover microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA5", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("failover microservice", "weather analysis microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA6", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("weather analysis microservice", "authentication microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA7", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("authentication microservice", "analytics microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA8", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("analytics microservice", "data synchronization microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA9", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("data synchronization microservice", "health monitoring microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA10", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("health monitoring microservice", "localization microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA11", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("localization microservice", "resource allocation microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA12", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("resource allocation microservice", "traffic control microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA13", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("traffic control microservice", "image processing microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA14", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("image processing microservice", "sensor integration microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA15", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("sensor integration microservice", "logistics and supply chain microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA16", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("logistics and supply chain microservice", "reporting and monitoring microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA17", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("reporting and monitoring microservice", "ai inference microservice", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA18", Tuple.UP, AppEdge.MODULE);
        application.addAppEdge("ai inference microservice", "user interface", minCpuLength + random.nextDouble(maxCpuLength - minCpuLength), 500, "FILTERED_DATA19", Tuple.UP, AppEdge.MODULE);
        
        
        application.addAppEdge("user interface", "client", 14, 500, "RESULT", Tuple.DOWN, AppEdge.MODULE);
        application.addAppEdge("client", "DISPLAY", 14, 500, "RESULT_DISPLAY", Tuple.DOWN, AppEdge.ACTUATOR);
        application.addAppEdge("client", "DISPLAY", 14, 500, "OBJECT_AVOIDANCE", Tuple.DOWN, AppEdge.ACTUATOR);
        
        //add modules with random parameters to the application according to the directed acyclic graph
        application.addTupleMapping("client", "MOTION SENSOR", "DATA", new FractionalSelectivity(0.9));
        application.addTupleMapping("communication microservice", "DATA", "FILTERED_DATA", new FractionalSelectivity(1.0));
        application.addTupleMapping("navigation microservice", "FILTERED_DATA", "MOTION_CONTROL", new FractionalSelectivity(1.0));
        application.addTupleMapping("telemetry microservice", "MOTION_CONTROL", "FILTERED_DATA2", new FractionalSelectivity(1.0));
        application.addTupleMapping("mission planning microservice", "FILTERED_DATA2", "FILTERED_DATA3", new FractionalSelectivity(1.0));
        application.addTupleMapping("collision avoidance microservice", "FILTERED_DATA3", "FILTERED_DATA4", new FractionalSelectivity(1.0));
        application.addTupleMapping("battery management microservice", "FILTERED_DATA4", "FILTERED_DATA5", new FractionalSelectivity(1.0));
        application.addTupleMapping("failover microservice", "FILTERED_DATA5", "FILTERED_DATA6", new FractionalSelectivity(1.0));
        application.addTupleMapping("weather analysis microservice", "FILTERED_DATA6", "FILTERED_DATA7", new FractionalSelectivity(1.0));
        application.addTupleMapping("authentication microservice", "FILTERED_DATA7", "FILTERED_DATA8", new FractionalSelectivity(1.0));
        application.addTupleMapping("analytics microservice", "FILTERED_DATA8", "FILTERED_DATA9", new FractionalSelectivity(1.0));
        application.addTupleMapping("data synchronization microservice", "FILTERED_DATA9", "FILTERED_DATA10", new FractionalSelectivity(1.0));
        application.addTupleMapping("health monitoring microservice", "FILTERED_DATA10", "FILTERED_DATA11", new FractionalSelectivity(1.0));
        application.addTupleMapping("localization microservice", "FILTERED_DATA11", "FILTERED_DATA12", new FractionalSelectivity(1.0));
        application.addTupleMapping("resource allocation microservice", "FILTERED_DATA12", "FILTERED_DATA13", new FractionalSelectivity(1.0));
        application.addTupleMapping("traffic control microservice", "FILTERED_DATA13", "FILTERED_DATA14", new FractionalSelectivity(1.0));
        application.addTupleMapping("image processing microservice", "FILTERED_DATA14", "FILTERED_DATA15", new FractionalSelectivity(1.0));
        application.addTupleMapping("sensor integration microservice", "FILTERED_DATA15", "FILTERED_DATA16", new FractionalSelectivity(1.0));
        application.addTupleMapping("logistics and supply chain microservice", "FILTERED_DATA16", "FILTERED_DATA17", new FractionalSelectivity(1.0));
        application.addTupleMapping("reporting and monitoring microservice", "FILTERED_DATA17", "FILTERED_DATA18", new FractionalSelectivity(1.0));
        application.addTupleMapping("ai inference microservice", "FILTERED_DATA18", "FILTERED_DATA19", new FractionalSelectivity(1.0));
        application.addTupleMapping("user interface", "FILTERED_DATA19", "RESULT", new FractionalSelectivity(1.0));
        application.addTupleMapping("client", "RESULT", "RESULT_DISPLAY", new FractionalSelectivity(1.0));
        application.addTupleMapping("client", "OBJECT_AVOIDANCE", "MOTION SENSOR", new FractionalSelectivity(1.0));

        //create runtime application execution loop
        final AppLoop loop1 = new AppLoop(new ArrayList<String> ()
        {{
            add("SENSOR");
            add("client");
            add("communication microservice");
            add("navigation microservice");
            add("telemetry microservice");
            add("mission planning microservice");
            add("collision avoidance microservice");
            add("battery management microservice");
            add("failover microservice");
            add("weather analysis microservice");
            add("authentication microservice");
            add("analytics microservice");
            add("data synchronization microservice");
            add("health monitoring microservice");
            add("localization microservice");
            add("resource allocation microservice");
            add("traffic control microservice");
            add("image processing microservice");
            add("sensor integration microservice");
            add("logistics and supply chain microservice");
            add("reporting and monitoring microservice");
            add("ai inference microservice");
            add("client");
            add("user interface");
        }}
        );

        //Create a list of loops and add loop1 to it
        List<AppLoop> loops = new ArrayList<AppLoop>()
        {{
            add(loop1);
        }};

        //Set the loops for the application
        application.setLoops(loops);
        return application;
    }
}