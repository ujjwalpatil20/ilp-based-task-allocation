from popper.util import Settings, format_prog
from popper.loop import learn_solution
import os
import sys

def run_popper(kb_path):
    print(f"Running Popper on {kb_path}")
    
    # Increase max rules and vars to find more complex solutions if needed
    settings = Settings(kbpath=kb_path, max_rules=2, max_vars=4, max_literals=4)
    
    # Run the learning loop
    result = learn_solution(settings)
    
    if result:
        # result is a tuple (prog, stats, ...)
        if isinstance(result, tuple):
             prog = result[0]
        else:
             prog = result
             
        if prog:
            print("\n\nSOLUTION FOUND:")
            print(format_prog(prog))
            
            # Simple explanation generator based on the found rule
            rule_str = format_prog(prog)
            explain_rule(rule_str)
            
            # Save the rule to a file for the allocator to use
            # We save it in the same directory as this script (ilp_learning)
            # OR better, one level up in task_management so it's easier to verify
            # Let's match the plan: save to src/task_management/learned_rules.pl
            
            # This script is in src/task_management/ilp_learning
            current_dir = os.path.dirname(os.path.abspath(__file__))
            parent_dir = os.path.dirname(current_dir)
            rules_file = os.path.join(parent_dir, 'learned_rules.pl')
            
            with open(rules_file, 'w') as f:
                f.write(f"% Learned from {kb_path}\n")
                f.write(f":- dynamic assigned/2.\n") 
                f.write(f"{rule_str}\n")
            print(f"Saved rule to {rules_file}")
        else:
            print("NO SOLUTION FOUND")
    else:
        print("NO RESULT RETURNED")

def explain_rule(rule_str):
    print("\n[EXPLANATION]")
    if "closest(V0,V1)" in rule_str and "idle(V0,V1)" in rule_str:
         print("The system assigns tasks to robots that are IDLE and are the CLOSEST to the target.")
    elif "closest(V0,V1)" in rule_str:
         print("The system assigns tasks to the robot that is CLOSEST to the target.")
    elif "most_charged(V0,V1)" in rule_str:
         print("The system assigns tasks to the robot that has the HIGHEST BATTERY.")
    elif "workload_bucket" in rule_str and "zero" in rule_str:
         print("The system prioritizes robots with ZERO WORKLOAD.")
    elif "workload_bucket" in rule_str and "low" in rule_str:
         print("The system prioritizes robots with LOW WORKLOAD.")
    else:
         print("The system has found a rule, but I don't have a canned explanation for it yet: " + rule_str)

if __name__ == "__main__":
    kb_path = os.path.dirname(os.path.abspath(__file__))
    if len(sys.argv) > 1:
        kb_path = sys.argv[1]
        
    try:
        run_popper(kb_path)
    except Exception as e:
        print(f"Error running popper: {e}")
