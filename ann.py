import numpy as np

class Ann:
    def __init__(self, genotypes):
        self.weights1, self.weights2 = self.extract_elements(genotypes)
        # self.weights2 = weights2
        # self.genotypes = genotypes
        # self.prev_output = [0, 0]
    
    def extract_elements(self, genotypes):
        all_first_seven = []
        all_two_values = []
        
        # First part: Extract the first 7 values for 14 iterations
        for i in range(14):
            first_seven = genotypes[i*7:(i*7)+7]  # Get 7 elements starting at index i*7
            all_first_seven.append(first_seven)  
            #print(f"Iteration {i+1}, first 7 values: {first_seven}")

        # Calculate the start index for the second part
        start_index = 14 * 7  # 14 iterations, each taking 7 elements

        # Second part: Extract 2 values for the next 7 iterations starting from where the first part ended
        for j in range(7):
            two_values = genotypes[start_index + j*2:start_index + (j*2) + 2]  # Get 2 elements starting at calculated start_index
            all_two_values.append(two_values)  
            #print(f"Iteration {j+1}, 2 values: {two_values}")

        first_seven_array = np.array(all_first_seven)
        two_values_array = np.array(all_two_values)
        
        return first_seven_array, two_values_array

    # def create_weights(self, size1, size2):
    #     weights = np.random.normal(0.0, 0.01, (size1, size2))
    #     return weights
    
    def relu(self, Z):

        """
        Implementation of the relu activation function
        :param Z: Tne value of the neuron
        :return: 0 if the neuron is turned off, Z otherwise
        """
        return np.maximum(0, Z)

    def sigmoid(self, x):
        return 1 / (1 + np.exp(-x))
    
    def tanh(self, x):
        return np.tanh(x)

    def feedforward(self, inputs):
        """
        Forward propagation through the network.
        """
        # self.layer1 = self.sigmoid(np.dot(inputs, self.weights1))
        # self.output = self.sigmoid(np.dot(self.layer1, self.weights2))
        self.layer1 = self.relu(np.dot(inputs, self.weights1))
        self.output = self.tanh(np.dot(self.layer1, self.weights2))
        return self.output

    def calculate_output(self, inputs):
        """
        Calculate the output for given input.
        """
        return self.feedforward(np.asarray(inputs))
    
if __name__ == "__main__":
    
    genotypes = []
    for i in range(112):
        genotypes.append(i)
    #Testing the output
    # Instantiating the Ann class with placeholders for 'layers' and 'genotypes'
    ann = Ann(genotypes)
    
    # Ensuring the input has the correct dimension (14 elements)
    inputs = np.array([20, 10, 15, 30, 40, 60, 50, 55, 5, 80, 90, 23, 0, 0])  # Added two zeros to match the input size
    prediction = ann.calculate_output(inputs)
    print("Prediction:", prediction)