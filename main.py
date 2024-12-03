import pygame
import sys
import json
from tkinter import W, Tk, filedialog

# Window dimensions
WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 600
from agent import Agent
from wall import Wall
from button import Button
from constants import (
    LEFT_BOUNDARY,
    RIGHT_BOUNDARY,
    TOP_BOUNDARY,
    BOTTOM_BOUNDARY,
    GREEN,
    RED,
    BLACK,
)
from controller_astar import ControllerAStar
from controller_basic import ControllerBasic
from controller_random import ControllerRandom
from text_input import TextInput

def load_environment_file_dialogue():
    """
    Uses the tkinter file dialogue to select the file to open.
    Calls open_walls after file selected.
    """
    global root
    root = Tk()
    root.withdraw()
    load_environment(
        filedialog.askopenfilename(
            defaultextension=".json", filetypes=[("JSON files", "*.json")]
        )
    )
    root.destroy()


def load_environment(filename):
    """
    Takes the file name and loads the environment.
    Puts all wall objects into the wall object and updates agent's internal memory.
    """
    walls = []
    agent = None

    if filename:
        with open(filename, "r") as f:
            world_data = json.load(f)

            # Load walls safely
            if "walls" in world_data:
                walls = [
                    Wall.from_dict(wall_data["wall"])
                    for wall_data in world_data["walls"]
                ]

            # Load agent safely
            if "agent" in world_data and world_data["agent"]:
                agent_data = world_data["agent"]["agent"]
                agent = Agent(
                    x=agent_data.get("x", 200),  # Default position if missing
                    y=agent_data.get("y", 200),
                    direction=agent_data.get("direction", 0),
                    walls=walls,
                    body_radius=agent_data.get("radius", 20),
                )
            else:
                # Default agent creation if no agent is defined
                agent = Agent(x=200, y=200, direction=0, walls=walls, body_radius=20)

    return walls, agent



def toggle_laser():
    """Thin wrapper to update whether to render LiDAR laser beams."""
    global agent
    agent.lidar_visible = not agent.lidar_visible


def toggle_controller_running():
    """Thin wrapper to update whether the controller is running or not."""
    global controller, text_surfaces
    controller.running = not controller.running
    text_surfaces[4] = font.render(
        "Controller ENABLED" if controller.running else "Controller DISABLED",
        True,
        GREEN if controller.running else RED,
    )
    print(f"Controller running: {controller.running}")  # Print state after toggling


def set_clock_rate():
    """Sets the clock rate to the value in the text input field."""
    global clock_rate, clock, text_surfaces
    try:
        clock_rate = int(clock_rate_input.get_text())
        text_surfaces[6] = font.render(f"Clock Rate: {clock_rate}", True, BLACK)
    except ValueError:
        pass


def set_max_speed():
    """Sets the clock rate to 0 (max speed) or back to the previous clock rate."""
    global clock_rate, clock, max_speed, text_surfaces, previous_clock_rate
    if max_speed:
        max_speed = False
        if previous_clock_rate > 0:  # Only restore if we had a valid previous rate
            clock_rate = previous_clock_rate
        else:
            clock_rate = 60  # Default to 60 if no valid previous rate
        text_surfaces[6] = font.render(f"Clock Rate: {clock_rate}", True, BLACK)
    else:
        max_speed = True
        if clock_rate > 0:  # Only save the previous rate if it's valid
            previous_clock_rate = clock_rate
        clock_rate = 0
        text_surfaces[6] = font.render("Clock Rate: MAX", True, BLACK)


# Initialize pygame
pygame.init()

# Define the clock for pygame
clock = pygame.time.Clock()

# Set up the display window
screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption("Simulation Window")

# Load in walls and agent
walls, agent = load_environment("worlds/test1.json")
# Assuming walls are loaded and you have a grid representation
grid_size = 20  # Define the size of the grid (adjust as necessary)
grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]


controller = ControllerAStar(agent, walls)

# Define clock rate variable
clock_rate = 60
previous_clock_rate = clock_rate
max_speed = False

# Define buttons
buttons = [
    Button(850, 50, 100, 50, "Load World", load_environment_file_dialogue),
    Button(850, 110, 100, 50, "See LiDAR", toggle_laser),
    Button(850, 170, 100, 50, "Controller", toggle_controller_running),
    Button(850, 230, 100, 50, "Set Clock Rate", set_clock_rate),
    Button(850, 290, 100, 50, "Max Speed", set_max_speed),
]

clock_rate_input = TextInput(x=960, y=230, width=50, height=50)

# Define on-screen text that renders in a block
font = pygame.font.Font(None, 24)
text_surfaces = [
    font.render("Load World Shortcut: u", True, BLACK),
    font.render("See LiDAR Shortcut: i", True, BLACK),
    font.render("Quit sim shortcut: q", True, BLACK),
    font.render("Toggle Controller: c", True, BLACK),
    font.render(
        "Controller ENABLED" if controller.running else "Controller DISABLED",
        True,
        GREEN if controller.running else RED,
    ),
    font.render("Move agent manually: Arrow Keys", True, BLACK),
    font.render(f"Clock Rate: {clock_rate}", True, BLACK),
    font.render("Actual Speed: Calculating...", True, BLACK),
]

# Main game loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                running = False
            if event.key == pygame.K_u:
                buttons[0].action()
            if event.key == pygame.K_i:
                buttons[1].action()
            if event.key == pygame.K_c:
                buttons[2].action()
            if event.key == pygame.K_m:
                buttons[4].action()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            for button in buttons:
                if button.is_clicked(event.pos):
                    button.action()
                print(f"Controller running: {controller.running}")
                mouse_pos = pygame.mouse.get_pos()
                controller.handle_input(mouse_pos)
                controller.set_goal(mouse_pos)

        clock_rate_input.handle_event(event)

    # Get the state of all keyboard buttons
    keys = pygame.key.get_pressed()

    # Handle agent's movement
    agent.handle_move_keys(keys)

    # Fill the screen with a white color
    screen.fill((255, 255, 255))

    # Draw the left half as the navigation area
    pygame.draw.rect(
        screen,
        (200, 200, 200),
        (
            LEFT_BOUNDARY,
            TOP_BOUNDARY,
            RIGHT_BOUNDARY - LEFT_BOUNDARY,
            BOTTOM_BOUNDARY - TOP_BOUNDARY,
        ),
    )

    # Agent scans environment
    agent.scan()

    # Controller does its work
    controller.update()

    # Draw the walls
    for wall in walls:
        wall.draw(screen)

    # Draw the agent
    agent.draw(screen)

    # Draw the buttons
    for button in buttons:
        button.draw(screen)

    # Draw text input
    clock_rate_input.update()
    clock_rate_input.draw(screen)

    # Draw text
    x, y = WINDOW_WIDTH - 300, WINDOW_HEIGHT - 200
    for surface in text_surfaces:
        screen.blit(surface, (x, y))
        y += 20

    # Update the display
    pygame.display.flip()

    # Control the frame rate and measure actual frame rate if at max speed
    if clock_rate > 0:
        clock.tick(clock_rate)
        actual_speed = clock_rate
    else:
        actual_speed = int(clock.get_fps())
        text_surfaces[7] = font.render(f"Actual Speed: {actual_speed} FPS", True, BLACK)
        clock.tick()

# Quit pygame
pygame.quit()
sys.exit()
