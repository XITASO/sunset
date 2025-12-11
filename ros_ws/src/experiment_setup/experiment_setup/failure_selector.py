import random

class CriticalitySelector:
    def __init__(self, data):
        self.data = data
        self.grouped_data = self.group_by_criticality()

    def group_by_criticality(self):
        # Group elements by their criticality level
        grouped = {}
        for element in self.data:
            criticality = element['criticality_level']
            if criticality not in grouped:
                grouped[criticality] = []
            grouped[criticality].append(element)
        return grouped

    def select_element(self, criticality):
        # Select an element based on probability within a given criticality
        if criticality not in self.grouped_data:
            raise ValueError(f"Criticality level '{criticality}' not found in data.")

        elements = self.grouped_data[criticality]
        #probabilities = [element['probability'] for element in elements]
        
        ## Randomly select an element based on their probabilities
        #selected_element = random.choices(elements, weights=probabilities, k=1)[0]
        selected_element = elements[0]
        return selected_element


if __name__ == '__main__':
    # Example usage:
    data_dict = [
    {"name": "Test", "probability": 0.1, "criticality_level": "Failure", "event": "Critical failure event"},
    {"name": "Test", "probability": 0.2, "criticality_level": "Failure", "event": "Minor failure event"},
    {"name": "Test", "probability": 0.7, "criticality_level": "Failure", "event": "Major failure event"},
    {"name": "Test", "probability": 0.3, "criticality_level": "Warning", "event": "Low priority warning"},
    {"name": "Test", "probability": 0.7, "criticality_level": "Warning", "event": "High priority warning"}
]
    selector = CriticalitySelector(data_dict)

    # Select an event from the "Failure" criticality level
    selected_failure = selector.select_element("Failure")
    print(selected_failure)

    # Select an event from the "Warning" criticality level
    selected_warning = selector.select_element("Warning")
    print(selected_warning)