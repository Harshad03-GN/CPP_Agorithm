import tkinter as tk
from tkinter import messagebox

# FSM Class
class FSM:
    def __init__(self):
        self.states = set()  # A set to hold all states
        self.alphabet = set()  # A set to hold all input symbols
        self.transitions = {}  # A dictionary for transitions
        self.start_state = None  # The start state
        self.accepting_states = set()  # A set to hold accepting states

    def set_states(self, states):
        self.states = set(states)

    def set_alphabet(self, alphabet):
        self.alphabet = set(alphabet)

    def set_start_state(self, start_state):
        if start_state in self.states:
            self.start_state = start_state
        else:
            raise ValueError("Start state must be one of the defined states.")

    def set_accepting_states(self, accepting_states):
        self.accepting_states = set(accepting_states)

    def add_transition(self, from_state, input_symbol, to_state):
        if from_state in self.states and to_state in self.states and input_symbol in self.alphabet:
            if from_state not in self.transitions:
                self.transitions[from_state] = {}
            self.transitions[from_state][input_symbol] = to_state
        else:
            raise ValueError("Invalid transition.")

    def test_input(self, input_string):
        current_state = self.start_state
        for symbol in input_string:
            if current_state in self.transitions and symbol in self.transitions[current_state]:
                current_state = self.transitions[current_state][symbol]
            else:
                return False
        return current_state in self.accepting_states


# GUI Class
class FSMGUI:
    def __init__(self, root):
        self.fsm = FSM()
        self.root = root
        self.root.title("FSM Designer and Tester")

        # State Section
        self.state_label = tk.Label(root, text="Enter States (comma separated):")
        self.state_label.grid(row=0, column=0)
        self.state_entry = tk.Entry(root)
        self.state_entry.grid(row=0, column=1)

        # Alphabet Section
        self.alphabet_label = tk.Label(root, text="Enter Alphabet (comma separated):")
        self.alphabet_label.grid(row=1, column=0)
        self.alphabet_entry = tk.Entry(root)
        self.alphabet_entry.grid(row=1, column=1)

        # Start State Section
        self.start_state_label = tk.Label(root, text="Enter Start State:")
        self.start_state_label.grid(row=2, column=0)
        self.start_state_entry = tk.Entry(root)
        self.start_state_entry.grid(row=2, column=1)

        # Accepting States Section
        self.accepting_states_label = tk.Label(root, text="Enter Accepting States (comma separated):")
        self.accepting_states_label.grid(row=3, column=0)
        self.accepting_states_entry = tk.Entry(root)
        self.accepting_states_entry.grid(row=3, column=1)

        # Transition Section
        self.transition_label = tk.Label(root, text="Enter Transitions (state1, symbol, state2):")
        self.transition_label.grid(row=4, column=0)
        self.transition_entry = tk.Entry(root)
        self.transition_entry.grid(row=4, column=1)

        self.add_transition_button = tk.Button(root, text="Add Transition", command=self.add_transition)
        self.add_transition_button.grid(row=4, column=2)

        # Test Input Section
        self.input_label = tk.Label(root, text="Enter Input String to Test:")
        self.input_label.grid(row=5, column=0)
        self.input_entry = tk.Entry(root)
        self.input_entry.grid(row=5, column=1)

        self.test_button = tk.Button(root, text="Test Input", command=self.test_input)
        self.test_button.grid(row=5, column=2)

        # Result Label
        self.result_label = tk.Label(root, text="Result: ", fg="blue")
        self.result_label.grid(row=6, column=0, columnspan=3)

        # Submit FSM Button
        self.submit_button = tk.Button(root, text="Submit FSM", command=self.submit_fsm)
        self.submit_button.grid(row=7, column=0, columnspan=3)

    def submit_fsm(self):
        try:
            states = self.state_entry.get().split(',')
            alphabet = self.alphabet_entry.get().split(',')
            start_state = self.start_state_entry.get()
            accepting_states = self.accepting_states_entry.get().split(',')

            self.fsm.set_states(states)
            self.fsm.set_alphabet(alphabet)
            self.fsm.set_start_state(start_state)
            self.fsm.set_accepting_states(accepting_states)

            messagebox.showinfo("FSM Setup", "FSM configuration has been submitted successfully!")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def add_transition(self):
        try:
            transition = self.transition_entry.get().split(',')
            if len(transition) != 3:
                raise ValueError("Transition must be in the format: state1,symbol,state2")
            from_state, input_symbol, to_state = transition
            self.fsm.add_transition(from_state.strip(), input_symbol.strip(), to_state.strip())
            self.transition_entry.delete(0, tk.END)
            messagebox.showinfo("Transition Added", "Transition added successfully!")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def test_input(self):
        input_string = self.input_entry.get()
        if self.fsm.test_input(input_string):
            self.result_label.config(text="Result: Accepted", fg="green")
        else:
            self.result_label.config(text="Result: Rejected", fg="red")


# Main Function to run the GUI
def main():
    root = tk.Tk()
    gui = FSMGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
