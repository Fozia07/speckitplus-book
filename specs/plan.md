# Implementation Plan: Physical AI & Humanoid Robotics Textbook

## 1. Executive Summary
- **Goal:** Transform specification documents (`specs/00*-module/spec.md`) into a deployed, interactive Docusaurus textbook.
- **Pedagogical Approach:** Emphasizing practical, hands-on learning through runnable code examples to bridge theory and application.
- **Strategy:** Linear execution of modules (1→4) with continuous CI/CD deployment, strictly adhering to Spec-Driven Development (SDD) principles.
- **Target Audience:** Students with basic programming knowledge and theoretical AI understanding.

## 2. Architecture & Infrastructure
- **Tech Stack:**
    - **Frontend:** Docusaurus v3 (React-based static site generator).
    - **Hosting:** GitHub Pages.
    - **Automation:** GitHub Actions for CI/CD.
    - **Language:** Markdown (Content), TypeScript/React (Custom Components), C++/Python/C# (Robotics Code).
- **Asset Strategy:**
    - Images/Diagrams stored centrally in `static/img`.
    - Runnable code examples stored in `code_examples/` to separate "lab code" from "textbook content".

## 3. Implementation Phases

### Phase 0: Infrastructure & Scaffolding
- [ ] **Scaffold Docusaurus:** Initialize project structure in `my-website/`.
- [ ] **Theme Setup:** Configure Docusaurus `docusaurus.config.ts` (title, tagline, deployment settings).
- [ ] **Folder Structure:** Create `docs/01-...`, `docs/02-...` directories matching the spec.
- [ ] **CI/CD Pipeline:** Configure GitHub Actions for automated deployment to `gh-pages` branch.
- [ ] **Sidebar Logic:** Set up `sidebars.ts` to auto-generate navigation based on folder structure.

### Phase 1: Module 1 - The Robotic Nervous System (ROS 2)
*Dependency: Phase 0*
- [ ] **Code Implementation:** Create basic ROS 2 packages in `code_examples/module-01-ros2/`.
- [ ] **Content Authoring:**
    - [ ] 01. ROS 2 Core Architecture
    - [ ] 02. Environment Setup
    - [ ] 03. Nodes & Executors
    - [ ] 04. Topics, Services, Actions
    - [ ] 05. Package Management
    - [ ] 06. Launch Files
    - [ ] 07. Debugging Tools
- [ ] **Validation:** Verify `ros2 run` commands works in examples.

### Phase 2: Module 2 - The Digital Twin (Gazebo & Unity)
*Dependency: Phase 1 (ROS 2 foundations)*
- [ ] **Code Implementation:**
    - Create URDF/SDF models in `code_examples/module-02-simulation/`.
    - Setup basic Unity project in `code_examples/module-02-unity/`.
- [ ] **Content Authoring:**
    - [ ] 01. Digital Twin Concepts
    - [ ] 02. Gazebo Setup
    - [ ] 03. Physics Basics
    - [ ] 04. URDF/SDF Modeling
    - [ ] 05. Sensor Simulation
    - [ ] 06. ROS 2 Control
    - [ ] 07. Unity Visualization
    - [ ] 08. Unity-ROS Bridge & URDF Import
- [ ] **Validation:** Verify simulation spawns and bridges to ROS 2.

### Phase 3: Module 3 - The AI-Robot Brain (NVIDIA Isaac)
*Dependency: Phase 1 & 2*
- [ ] **Code Implementation:** Isaac Sim python scripts in `code_examples/module-03-isaac/`.
- [ ] **Content Authoring:**
    - [ ] 01. Isaac Ecosystem
    - [ ] 02. Isaac Sim Env
    - [ ] 03. Synthetic Data Gen
    - [ ] 04. Isaac ROS Architecture
    - [ ] 05. Perception Pipelines
    - [ ] 06. VSLAM
    - [ ] 07. Navigation
    - [ ] 08. Jetson Deployment
- [ ] **Validation:** Verify Isaac Sim scripts run.

### Phase 4: Module 4 - Vision-Language-Action (VLA)
*Dependency: Phase 3 (Brain) & 1 (Nervous System)*
- [ ] **Code Implementation:** VLA integration scripts in `code_examples/module-04-vla/`.
- [ ] **Content Authoring:**
    - [ ] 01. VLA Concepts
    - [ ] 02. Whisper Voice Input
    - [ ] 03. Vision Perception
    - [ ] 04. LLM Task Planning
    - [ ] 05. Lang-to-Action
    - [ ] 06. Multi-Modal Arch
    - [ ] 07. Safety Systems
- [ ] **Validation:** Verify end-to-end VLA pipeline flow.

## 4. Component Map & Directory Structure

```text
my-website/
├── docusaurus.config.ts        # Configuration
├── sidebars.ts                 # Sidebar navigation logic
├── src/
│   ├── components/             # Custom React Components
│   └── css/
│       └── custom.css          # Global styling
├── static/
│   └── img/                    # Images & Diagrams
├── docs/                       # TEXTBOOK CONTENT
│   ├── 01-ros2-system/         # Module 1 Content
│   ├── 02-digital-twin/        # Module 2 Content
│   ├── 03-isaac-brain/         # Module 3 Content
│   └── 04-vla-systems/         # Module 4 Content
└── code_examples/              # RUNNABLE CODE (Linked from docs)
    ├── module-01-ros2/
    ├── module-02-unity/
    ├── module-03-isaac/
    └── module-04-vla/
```

## 5. Quality Assurance & Validation
- **Traceability Check:** Every chapter in `docs/` must map to a spec item in `specs/`.
- **Build Check:** `npm run build` must pass before any merge.
- **Link Check:** Automated check for broken internal/external links.
- **Code Verification:** All snippets in `code_examples/` must be syntactically correct and run in their specified environment.
