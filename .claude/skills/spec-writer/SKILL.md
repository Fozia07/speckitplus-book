---
name: Spec Writer
description: Creates clear, structured technical specifications following Spec-Driven Development principles, including objectives, success criteria, constraints, and out-of-scope items.
version: "1.0"
---

## When to Use

Use this skill when you need to write a new technical specification for a feature, module, or system component, especially when following Spec-Driven Development (SDD) practices. It ensures clarity, measurability, and well-defined boundaries.

## Step-by-Step Spec Creation Process

1.  **Define Core Objective:** Clearly state the primary goal of the feature/system in one or two sentences.
2.  **Identify In-Scope Items:** Detail all functionalities, components, and user interactions that are part of this specification.
3.  **Define Out-of-Scope Items:** Explicitly list what will NOT be included or addressed in this iteration to manage expectations and scope creep.
4.  **Establish Constraints:** Document any technical, operational, security, or business limitations that must be adhered to.
5.  **Formulate Success Criteria:** Define clear, measurable conditions that, when met, indicate the successful delivery of the specification's objective.
6.  **Structure and Format:** Organize the content logically using headings, subheadings, and bullet points to ensure readability and maintain structured formatting.
7.  **Review and Validate:** Use the validation checklist to ensure all critical aspects are covered and the spec is unambiguous.

## Validation Checklist

-   [ ] **Clear Objective:** Is the primary goal of the spec clearly articulated?
-   [ ] **In-Scope Defined:** Are all functionalities and components explicitly listed?
-   [ ] **Out-of-Scope Explicit:** Is it clear what is NOT part of this spec?
-   [ ] **Constraints Documented:** Are all known limitations and boundaries stated?
-   [ ] **Measurable Success Criteria:** Are the success conditions clear and quantifiable?
-   [ ] **Structured Format:** Is the document well-organized and easy to read?
-   [ ] **Unambiguous Language:** Is there any room for misinterpretation?

## Output Template

```markdown
# [Feature/Module Name] Specification

## 1. Objective
[A concise statement of what this feature/module aims to achieve.]

## 2. Scope

### 2.1. In Scope
- [List of functionalities, components, or user stories that are included.]
- [Example: User registration process]
- [Example: Basic profile management]

### 2.2. Out of Scope
- [List of items explicitly excluded from this specification.]
- [Example: Password recovery (will be handled in a separate iteration)]
- [Example: Integration with third-party authentication providers]

## 3. Constraints
- [Any technical limitations (e.g., must use existing database, specific API version).]
- [Any operational constraints (e.g., deployment environment).]
- [Any security constraints (e.g., must comply with industry standards).]
- [Example: Must integrate with existing user management system.]
- [Example: Response times for critical operations must be < 100ms.]

## 4. Success Criteria
- [Clear, measurable conditions for successful implementation.]
- [Example: Users can register an account successfully and log in.]
- [Example: All defined in-scope API endpoints return correct data with < 2% error rate.]
- [Example: All acceptance tests for in-scope features pass.]

## 5. Architectural Considerations (Optional - for high-level specs)
[Brief notes on potential architectural impacts or decisions to be made during planning.]

```
