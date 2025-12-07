---
name: Task Creator
description: Creates a structured list of atomic, testable tasks from a high-level goal, incorporating acceptance criteria and dependencies for effective project planning.
version: "1.0"
---

## When to Use

Use this skill when you need to break down a high-level goal, feature, bug fix, or project into actionable, manageable, and atomic tasks. It is particularly useful for planning implementation, identifying dependencies, and ensuring clear completion conditions.

## Step-by-Step Task Creation Process

1.  **Understand the High-Level Goal:** Clearly articulate the main objective or outcome to be achieved.
2.  **Break Down into Major Sections/Phases:** Divide the overall goal into logical, higher-level sections or phases (e.g., Research, Design, Implementation, Testing, Deployment, Documentation).
3.  **Identify Atomic Tasks within Each Section:** For each major section, list individual, focused tasks that can be completed in a relatively short timeframe (e.g., 15-60 minutes). Each task should represent a single unit of work.
4.  **Define Clear Acceptance Criteria:** For every atomic task, specify explicit, measurable conditions that must be met for the task to be considered complete. These should be testable.
5.  **Establish Dependencies (if any):** Note any tasks or phases that must be completed before another can begin.
6.  **Review and Refine:** Evaluate the generated task list for completeness, clarity, atomicity, logical flow, and ensure all acceptance criteria are precise. Adjust as necessary.

## Quality Criteria

-   **Atomic Tasks:** Each task describes a single, focused unit of work.
-   **Clear Acceptance Criteria:** Every task includes unambiguous and testable conditions for completion.
-   **Logical Flow:** Tasks and phases are ordered in a sensible and progressive manner.
-   **Comprehensive Coverage:** The task list addresses all essential aspects required to achieve the high-level goal.
-   **Actionable:** Tasks are concrete, specific, and can be immediately acted upon without further decomposition.

## Output Template

```markdown
# [High-Level Goal/Project Name] Tasks

## Phase 1: [Phase Name]

1.  **Task: [Atomic Task Description]**
    *   **Acceptance Criteria:** [Measurable condition for completion]
    *   **Dependencies:** [Optional: List preceding tasks/phases]

2.  **Task: [Atomic Task Description]**
    *   **Acceptance Criteria:** [Measurable condition for completion]

## Phase 2: [Phase Name]

1.  **Task: [Atomic Task Description]**
    *   **Acceptance Criteria:** [Measurable condition for completion]
    *   **Dependencies:** [Optional: List preceding tasks/phases, e.g., "Phase 1 Completion"]

... (continue for all phases and tasks)

---
**Final Review & Approval**
**Human Review Required**
**Continue on Approval**
---
```
