#!/bin/bash
# Setup Development Workspace for Grunt Platform
#
# This script clones the standard Grunt support repositories into a ROS 2 workspace
# and prepares it for building inside the grunt container.
#
# Usage:
#   ./tools/setup-dev-workspace.sh [DISTRO]
#
# Arguments:
#   DISTRO    ROS distribution (humble or jazzy), defaults to humble
#
# Example:
#   ./tools/setup-dev-workspace.sh humble
#   ./tools/setup-dev-workspace.sh jazzy

set -e  # Exit on error

# Configuration
DISTRO="${1:-humble}"
WORKSPACE_ROOT="$HOME/ros2/${DISTRO}/dev_ws"
GITHUB_USER="pondersome"

# Color output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Grunt Dev Workspace Setup${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${GREEN}ROS Distro:${NC} ${DISTRO}"
echo -e "${GREEN}Workspace:${NC} ${WORKSPACE_ROOT}"
echo -e "${GREEN}GitHub User:${NC} ${GITHUB_USER}"
echo ""

# Verify distro is valid
if [[ "$DISTRO" != "humble" && "$DISTRO" != "jazzy" ]]; then
    echo -e "${RED}Error: Invalid ROS distro '${DISTRO}'. Must be 'humble' or 'jazzy'.${NC}"
    exit 1
fi

# Check if parent directory exists and has correct permissions
WORKSPACE_PARENT="$HOME/ros2/${DISTRO}"
if [[ -d "$HOME/ros2" ]]; then
    OWNER=$(stat -c '%U' "$HOME/ros2" 2>/dev/null || stat -f '%Su' "$HOME/ros2" 2>/dev/null)
    if [[ "$OWNER" != "$USER" ]]; then
        echo -e "${RED}Error: $HOME/ros2 is owned by '${OWNER}', not '${USER}'${NC}"
        echo -e "${YELLOW}This commonly happens when Docker Compose creates the directory.${NC}"
        echo ""
        echo -e "${GREEN}Fix with:${NC}"
        echo -e "  sudo chown -R ${USER}:${USER} $HOME/ros2"
        echo ""
        exit 1
    fi
fi

# Create workspace directory structure
echo -e "${BLUE}[1/3] Creating workspace structure...${NC}"
mkdir -p "${WORKSPACE_ROOT}/src"

# Clone repositories using vcs tool
echo -e "${BLUE}[2/3] Cloning repositories using vcstool...${NC}"
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPOS_FILE="${SCRIPT_DIR}/grunt_repos.yaml"

# Check if repos file exists
if [[ ! -f "$REPOS_FILE" ]]; then
    echo -e "${RED}Error: Repository list file not found: ${REPOS_FILE}${NC}"
    exit 1
fi

# Check if vcs is installed
if ! command -v vcs &> /dev/null; then
    echo -e "${YELLOW}Warning: vcstool not found. Installing...${NC}"
    pip3 install --user vcstool
    # Add user's local bin to PATH if not already there
    export PATH="$HOME/.local/bin:$PATH"
fi

# Import repositories
cd "${WORKSPACE_ROOT}/src"
echo -e "${GREEN}Using repository list: ${REPOS_FILE}${NC}"
vcs import < "${REPOS_FILE}"

echo ""
echo -e "${GREEN}Repository import complete!${NC}"

echo ""
echo -e "${BLUE}[3/3] Workspace setup complete!${NC}"
echo ""
echo -e "${GREEN}Repositories cloned:${NC}"
ls -1 "${WORKSPACE_ROOT}/src/"
echo ""

# Display next steps
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Next Steps${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "1. ${GREEN}Start the bash container:${NC}"
echo -e "   docker compose -f compose/viz/bash.yaml run --rm bash"
echo ""
echo -e "2. ${GREEN}Inside the container, build the workspace:${NC}"
echo -e "   cd ~/dev_ws"
echo -e "   colcon build --symlink-install"
echo -e "   source install/setup.bash"
echo ""
echo -e "3. ${GREEN}Test your packages:${NC}"
echo -e "   ros2 pkg list | grep grunt"
echo -e "   ros2 pkg list | grep roarm"
echo ""
echo -e "4. ${GREEN}Launch RViz with workspace packages:${NC}"
echo -e "   (In another terminal)"
echo -e "   docker compose -f compose/viz/rviz.yaml up"
echo ""
echo -e "${YELLOW}Note:${NC} The workspace is bind-mounted, so changes made on the host"
echo -e "      will be visible in the container and vice versa."
echo ""
