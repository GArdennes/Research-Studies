package tech.skargen.recloud.developers.cypherskar.pso;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import org.cloudbus.cloudsim.Cloudlet;
import org.cloudbus.cloudsim.Vm;
import tech.skargen.recloud.components.cloudsim.IReBroker;
import tech.skargen.recloud.components.simulation.ASimulation;

public class FireflyAlgorithm extends ASimulation {
    private int numFireflies;
    private int maxIterations;
    private double alpha;
    private double beta;
    private double gamma;
    private int numIterations;
    private List<Vm> vms;
    private List<Cloudlet> cloudlets;

    public FireflyAlgorithm(int numFireflies, int maxIterations, double alpha, double beta, double gamma
                            /*List<Vm> vms, List<Cloudlet> cloudlets*/) {
        this.numFireflies = numFireflies;
        this.maxIterations = maxIterations;
        this.alpha = alpha;
        this.beta = beta;
        this.gamma = gamma;
        //this.vms = vms;
        //this.cloudlets = cloudlets;
        this.numIterations = maxIterations;
    }

    @Override
    public String getAlgorithmName() {
        return "Firefly";
    }

    @Override
    public String getDeveloper() {
        return "Kevin";
    }

    @Override
    public String getAdditionalInfo() {
        return null;
    }

    @Override
    public void startEntity(IReBroker rebroker) {
    }

    @Override
    public void shutdownEntity(IReBroker rebroker) {
    }

    @Override
    public void processCloudletsSubmit(IReBroker rebroker) {
        List<Cloudlet> cloudlets = rebroker.getEntity().getCloudletList();
        List<Vm> vms = rebroker.getEntity().getVmsCreatedList();

        rebroker.updateProgressMax(numIterations);
        rebroker.updateProgressMessage("Firefly Algorithm By github.com/GArdennes");

        int[] result = findSolution(cloudlets, vms, rebroker);

        for (int i = 0; i < result.length; i++) {
            cloudlets.get(i).setVmId(vms.get(result[i]).getId());
            rebroker.submitCloudlet(cloudlets.get(i));
        }
    }

    @Override
    public <T extends Cloudlet> void processCloudletReturn(IReBroker rebroker, T task) {
    }
    
    
    public void run() {
        // Generate initial population of fireflies
        List<Firefly> population = generateInitialPopulation();

        // Perform firefly algorithm iterations
        for (int iteration = 1; iteration <= maxIterations; iteration++) {
            updateFireflies(population);
            evaluateFireflies(population);
            sortFireflies(population);

            // Print the best fitness value in each iteration
            double bestFitness = population.get(0).getFitness();
            System.out.println("Iteration " + iteration + ": Best Fitness = " + bestFitness);

            // Update fireflies' positions based on attractiveness
            updateFirefliesPositions(population);
        }
    }

    private List<Firefly> generateInitialPopulation() {
        // Create a random number generator
        Random random = new Random();

        // Create the initial population of fireflies
        List<Firefly> population = new ArrayList<>();
        for (int i = 0; i < numFireflies; i++) {
            double[] position = new double[vms.size()];
            for (int j = 0; j < vms.size(); j++) {
                // Generate a random position within the bounds of the VM
                Vm vm = vms.get(j);
                double min = vm.getMips();
                double max = vm.getMips();
                position[j] = min + (max - min) * random.nextDouble();
            }
            Firefly firefly = new Firefly(position);
            population.add(firefly);
        }

        return population;
    }

    private void updateFireflies(List<Firefly> population) {
        // Update fireflies' attractiveness values based on their distances
        for (int i = 0; i < population.size(); i++) {
            Firefly currentFirefly = population.get(i);
            for (int j = 0; j < population.size(); j++) {
                if (i != j) {
                    Firefly otherFirefly = population.get(j);
                    double distance = currentFirefly.calculateDistance(currentFirefly.getPosition(), otherFirefly.getPosition());
                    double attractiveness = calculateAttractiveness(distance);
                    currentFirefly.updateAttractiveness(attractiveness);
                }
            }
        }
    }

    

    private double calculateAttractiveness(double distance) {
        // Calculate the attractiveness based on the distance
        return beta * Math.exp(-gamma * distance * distance);
    }

    private void evaluateFireflies(List<Firefly> population) {
        // Evaluate the fitness of each firefly
        for (Firefly firefly : population) {
            double[] solution = firefly.getPosition();
            double fitness = evaluateSolution(solution);
            firefly.setFitness(fitness);
        }
    }

    private double evaluateSolution(double[] solution) {
        // Use the IReBroker and cloudlets to evaluate the fitness of the solution
        // Implement your evaluation logic here
        // Example: Return the total processing time of the cloudlets on the VMs
        double fitness = 0.0;
        for (int i = 0; i < cloudlets.size(); i++) {
            Cloudlet cloudlet = cloudlets.get(i);
            Vm vm = vms.get((int) solution[i]);
            fitness += cloudlet.getCloudletTotalLength() / vm.getMips();
        }
        return fitness;
    }

    private void sortFireflies(List<Firefly> population) {
        // Sort the fireflies based on their fitness values in descending order
        population.sort(Comparator.comparingDouble(Firefly::getFitness).reversed());
    }

    private void updateFirefliesPositions(List<Firefly> population) {
        // Update fireflies' positions based on their attractiveness and randomization
        Random random = new Random();
        for (int i = 0; i < population.size(); i++) {
            Firefly currentFirefly = population.get(i);
            for (int j = 0; j < population.size(); j++) {
                if (i != j) {
                    Firefly otherFirefly = population.get(j);
                    double[] currentPosition = currentFirefly.getPosition();
                    double[] otherPosition = otherFirefly.getPosition();
                    double[] newPosition = new double[currentPosition.length];
                    for (int k = 0; k < currentPosition.length; k++) {
                        double diff = otherPosition[k] - currentPosition[k];
                        newPosition[k] = currentPosition[k] + alpha * random.nextDouble() * diff;
                    }
                    currentFirefly.setPosition(newPosition);
                }
            }
        }
    }

    private class Firefly {
        private double[] position;
        private double fitness;
        private double attractiveness;

        public Firefly(double[] position) {
            this.position = position;
            this.fitness = 0.0;
            this.attractiveness = 0.0;
        }

        public double[] getPosition() {
            return position;
        }

        public double getFitness() {
            return fitness;
        }
        
        public double calculateDistance(double[] position1, double[] position2) {
            double distanceSquared = 0.0;
            for (int i = 0; i < position1.length; i++) {
                double diff = position1[i] - position2[i];
                distanceSquared += diff * diff;
            }
            return Math.sqrt(distanceSquared);
        }

        public double getAttractiveness() {
            return attractiveness;
        }

        public void setFitness(double fitness) {
            this.fitness = fitness;
        }

        public void updateAttractiveness(double attractiveness) {
            this.attractiveness += attractiveness;
        }
        
        public void setPosition(double[] position) {
            this.position = position;
        }
        
    }

    private int[] findSolution(List<Cloudlet> cloudlets, List<Vm> vms, IReBroker rebroker) {
        // Implement your findSolution logic here
        // Example: Return a random assignment of cloudlets to VMs
        Random random = new Random();
        int[] assignment = new int[cloudlets.size()];
        for (int i = 0; i < cloudlets.size(); i++) {
            assignment[i] = random.nextInt(vms.size());
        }
        return assignment;
    }
}
