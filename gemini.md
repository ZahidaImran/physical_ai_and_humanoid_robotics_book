# hackathon_book Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-06

## Active Technologies
- Python 3.9+, C++17 (for ROS 2 examples) + Docusaurus, ROS 2, Gazebo, Unity, NVIDIA Isaac Sim (1-physical-ai-robotics-book)
- N/A (Content stored in Git) (1-physical-ai-robotics-book)

[UNKNOWN]

## Project Structure

```text
[UNKNOWN]
```

## Commands

[UNKNOWN]

## Code Style

[UNKNOWN]

## Recent Changes
- 1-physical-ai-robotics-book: Added Python 3.9+, C++17 (for ROS 2 examples) + Docusaurus, ROS 2, Gazebo, Unity, NVIDIA Isaac Sim

[UNKNOWN]

<!-- MANUAL ADDITIONS START -->

## Gemini Agent Commands

This section defines the custom commands for the Gemini CLI agent to interact with this project.

| Command | PowerShell Script | Description |
|---|---|---|
| `/sp.specify` | `.specify/scripts/powershell/create-new-feature.ps1` | Creates a new feature branch and specification file. Requires a feature name as an argument. |
| `/sp.plan` | `.specify/scripts/powershell/setup-plan.ps1` | Generates the implementation plan for the current feature. |
| `/sp.tasks` | *Not yet implemented* | Generates the task list for the current feature. |
| `/sp.checklist` | *Not yet implemented* | Generates a checklist for the current feature. |
| `/sp.update` | `.specify/scripts/powershell/update-agent-context.ps1`| Updates the agent context files. |
| `/sp.implement` | *Not yet implemented* | Do the implementation plan for the current feature. |


To execute a command, I will run the corresponding PowerShell script using the `run_shell_command` tool.

<!-- MANUAL ADDITIONS END -->
