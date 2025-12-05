# Cognitive Planning: LLM Planning of Sequences

Beyond simple, direct voice commands, the true power of Vision-Language-Action (VLA) models for humanoid robotics emerges when Large Language Models (LLMs) are used for high-level cognitive planning. This involves translating complex, abstract natural language instructions into a sequence of executable robot actions. Instead of explicitly programming every step, an LLM can infer the user's intent, break it down into sub-goals, and generate a logical plan for the robot to follow.

This chapter will delve into the concept of cognitive planning using LLMs for humanoid robots. We'll explore how LLMs can act as the "brain" for high-level task decomposition and symbolic reasoning, bridging the gap between human language and robot execution.

## The Need for Cognitive Planning

Traditional robotics often relies on pre-programmed sequences or elaborate state machines for task execution. However, this approach struggles with:

*   **Ambiguity**: Human instructions are inherently ambiguous and lack explicit detail for robots.
*   **Novelty**: Robots need to adapt to new situations and tasks not explicitly coded.
*   **Scalability**: Manually programming every possible task permutation is unfeasible.

LLMs, with their vast world knowledge and reasoning capabilities, offer a solution by enabling robots to perform:

*   **Task Decomposition**: Breaking a high-level goal ("clean the table") into a series of smaller, manageable steps ("identify objects," "pick up cup," "move to sink," etc.).
*   **Symbolic Reasoning**: Understanding relationships between objects and actions ("a cup can hold water," "cleaning involves removing items").
*   **Error Recovery**: Adapting plans if a step fails.
*   **Human-Robot Dialogue**: Engaging in clarifying conversations about the task.

## LLMs as Robot Planners

An LLM can serve as a high-level planner in a hierarchical robotics architecture.

### Architecture Overview:

1.  **Human Instruction**: User provides a natural language instruction (e.g., "Please get me a drink from the fridge").
2.  **LLM (Cognitive Planner)**:
    *   Receives the instruction.
    *   Accesses a "robot API" or a knowledge base of available robot skills (e.g., `move_to(location)`, `grasp(object)`, `open(door)`).
    *   Generates a sequence of these low-level robot skills/actions to achieve the high-level goal.
    *   Can perform reasoning (e.g., "to get a drink, I need to open the fridge, pick up a bottle, close the fridge").
3.  **ROS 2 Action Sequencer (Low-Level Executor)**: Receives the sequence of robot skills from the LLM.
    *   Translates each skill into specific ROS 2 action goals, service calls, or topic commands.
    *   Executes these commands on the robot.
    *   Provides feedback on execution status back to the LLM (for replanning if needed).
4.  **Robot Hardware/Simulation**: Executes the low-level commands.
5.  **Perception Feedback**: Robot sensors provide real-time information to the LLM (e.g., "fridge door is open," "grasped bottle successfully").

## Prompt Engineering for Robot Planning

Effectively utilizing LLMs for planning requires careful prompt engineering. The prompt given to the LLM needs to define:

*   **Robot's Capabilities**: A list of available actions/skills the robot can perform, along with their parameters and preconditions.
*   **Environment State**: Current information about the robot and its environment (e.g., known objects, robot's location).
*   **User's Goal**: The high-level natural language instruction.
*   **Constraints**: Any limitations or safety considerations.

### Example Prompt Structure:

```
You are a helpful robot assistant. Your goal is to generate a sequence of actions to fulfill human requests.

Available actions:
- move_to(location): Moves the robot to a specified location. Locations are: kitchen, living_room, fridge, table.
- grasp(object): Grasps an object. Objects are: cup, bottle.
- open(door_name): Opens a specified door. Doors are: fridge_door.
- close(door_name): Closes a specified door. Doors are: fridge_door.

Current environment state:
- Robot is in the living_room.
- Fridge is closed.
- Table has a cup.

Human Request: "Please get me a bottle from the fridge."

Generate a concise sequence of actions, one per line.
```

The LLM might then respond with a sequence like:

```
move_to(fridge)
open(fridge_door)
grasp(bottle)
close(fridge_door)
move_to(table)
```

This sequence can then be parsed and executed by the ROS 2 Action Sequencer.

## Challenges in LLM-based Planning:

*   **Grounding**: Ensuring the LLM's abstract plan maps correctly to the robot's physical capabilities and the real world.
*   **Hallucinations**: LLMs can generate plausible but physically impossible or incorrect plans.
*   **Robustness**: Handling unexpected events or changes in the environment that deviate from the LLM's initial plan.
*   **Safety**: Ensuring generated plans are safe and do not lead to dangerous robot behaviors.
*   **Computational Cost**: Running large LLMs in real-time on robot hardware can be challenging.

## Conclusion

LLM-based cognitive planning offers a paradigm shift in humanoid robotics, enabling robots to understand and execute complex natural language instructions. By acting as high-level planners, LLMs can decompose tasks, reason about the environment, and generate executable action sequences, bringing us closer to truly intelligent and adaptable humanoid agents. The next chapter will explore how these planned sequences are executed using ROS 2 action sequencing.
