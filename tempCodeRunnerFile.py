w = [random.uniform(0, 10) for _ in range(6)]

J_e = robot.jacobe(q)
effort_robotics_toolbox = np.round(robot.pay(w, q, J_e), 6)
effort_my_equation = np.round(computeEffortHW3(q, w), 6)

with np.printoptions(precision=6, suppress=True, floatmode='fixed'):
    print("3) Test = ")
    print("random w")
    print(w)
    print("Tau form robotics_toolbox")
    print(effort_robotics_toolbox)  # force from robot
    print("Tau form my equation")
    print(effort_my_equation)  # force from external force
    print("effort_robotics_toolbox + effort_my_equation = ")
    print(effort_robotics_toolbox + effort_my_equation)
    print("-----------------------------------------------")