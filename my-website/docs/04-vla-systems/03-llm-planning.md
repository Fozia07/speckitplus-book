---
sidebar_position: 3
title: LLMs as Planners
---

# LLMs as Action Planners

Once a robot "sees" (via VLM), it needs to "think." Large Language Models (LLMs) act as the reasoning engine, translating high-level, ambiguous user instructions into structured plans.

## 1. The Reasoning Gap

*   **User Command:** "Clean up the table."
*   **Robot Capability:** `move_to(x,y)`, `pick(obj)`, `place(obj)`.
*   **The LLM's Job:** Decompose the command:
    1.  Detect objects on the table.
    2.  Identify which are "trash" or "misplaced."
    3.  Generate a sequence of `pick` and `place` calls.

## 2. Hands-On: Text-to-Action

We explore this in `code_examples/module-04-vla/scripts/llm_action_planner.py`.

### Defining the API
The LLM must know what the robot *can* do. We provide this as a context prompt or a function schema (like OpenAI Function Calling).

```python
robot_api = {
    "move_to_location": ["location_name"],
    "pick_up_object": ["object_name"],
    "place_object": ["object_name", "location_name"],
    "scan_area": []
}
```

### The Planning Logic
When prompted with *"Move the box from here to there"*, the LLM (simulated) generates:

```json
[
  {"action": "scan_area", "args": []},
  {"action": "pick_up_object", "args": ["box"]},
  {"action": "move_to_location", "args": ["destination_area"]},
  {"action": "place_object", "args": ["box", "destination_area"]}
]
```

## 3. Prompt Engineering for Robots
To make LLMs reliable planners, we use specific techniques:

*   **Chain of Thought (CoT):** Asking the LLM to "think step-by-step" before outputting JSON.
    *   *Example:* "First, I need to find the box. I see it on the table. Now I will pick it up..."
*   **In-Context Learning:** Providing examples of valid plans in the prompt.
    *   *User:* "Get the apple." -> *Plan:* `[scan, pick(apple)]`.
    *   *User:* "Get the banana." -> *Plan:* ...

## 4. Challenges
*   **Hallucination:** The LLM might invent a function `fly_to_ceiling()` that the robot doesn't have.
*   **Physical Feasibility:** The LLM doesn't know physics. It might ask the robot to pick up a table that is too heavy.
