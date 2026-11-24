"""Launch the simulation with the appropriate interpreter for this platform."""

from pathlib import Path
import platform
import subprocess
import sys


def main():
    command = "mjpython" if platform.system() == "Darwin" else "python"
    # Headless simulation does not need the macOS viewer interpreter.
    if "--headless" in sys.argv[1:]:
        command = sys.executable
    simulation = Path(__file__).resolve().with_name("simulation.py")
    try:
        subprocess.run([command, str(simulation), *sys.argv[1:]], check=True)
    except FileNotFoundError:
        sys.exit(
            f"Cannot find {command}. Activate your virtual environment and install "
            "scout_mini/requirements.txt."
        )
    except subprocess.CalledProcessError as error:
        sys.exit(error.returncode)
    except KeyboardInterrupt:
        sys.exit(130)


if __name__ == "__main__":
    main()
