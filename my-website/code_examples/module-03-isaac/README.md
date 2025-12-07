# NVIDIA Isaac Sim Setup & Examples

## Prerequisites
1.  **NVIDIA GPU**: RTX 2070 or higher recommended.
2.  **OS**: Ubuntu 20.04/22.04 or Windows 10/11.
3.  **NVIDIA Driver**: Latest Studio or Game Ready Driver.
4.  **Omniverse Launcher**: Download from NVIDIA website.

## Setup Steps
1.  **Install Omniverse Launcher**: Run the installer and log in with your NVIDIA account.
2.  **Install Cache & Nucleus**: Go to the "Exchange" tab and install "Nucleus" (local server for assets).
3.  **Install Isaac Sim**: Search for "Isaac Sim" in the Exchange tab and install the latest version.
4.  **VS Code Support**: In Isaac Sim, go to `Window` -> `Extensions` and enable `omni.isaac.vscode` for better debugging.

## Running the Examples

### Hello World Script
This script launches a standalone Isaac Sim application, creates a world, and drops a red cube (our "robot") affected by gravity.

1.  Open the "Isaac Sim" terminal (or use the python.sh/python.bat provided by the Isaac Sim installation).
2.  Run:
    ```bash
    ./python.sh path/to/hello_world_isaac.py
    ```
