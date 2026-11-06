"""Fixed timestep update loop with pygame-ce, in the shared look of the demos (JetBrains Mono, teal and orange).

Runs natively (`python main.py`) and in the browser through Pyodide.
"""
import asyncio
import os
import sys

import pygame

from stepper import FixedStepper

WIDTH, HEIGHT, SIZE = 960, 540, 64
LEFT, RIGHT = 40, WIDTH - 40 - SIZE
STEP = 1 / 60

TEAL, ORANGE, INK = (13, 148, 136), (234, 88, 12), (17, 24, 39)
MUTED, PAPER, GRID, WHITE = (107, 114, 128), (250, 250, 249), (231, 229, 228), (255, 255, 255)
TEAL_SOFT = (204, 251, 241)


def font_path(name):
    """Browser: the TTFs sit next to main.py. Native: they live in common/fonts."""
    here = os.path.dirname(os.path.abspath(__file__))
    for folder in (here, os.path.join(here, "..", "common", "fonts")):
        path = os.path.join(folder, name)
        if os.path.exists(path):
            return path
    return None


def card(screen, rect, fill, edge=INK):
    """Rectangle with a hard offset shadow, like the buttons on malura.de."""
    shadow = pygame.Surface((rect[2], rect[3]), pygame.SRCALPHA)
    shadow.fill((17, 24, 39, 40))
    screen.blit(shadow, (rect[0] + 4, rect[1] + 4))
    pygame.draw.rect(screen, fill, rect)
    pygame.draw.rect(screen, edge, rect, 1)


def backdrop(screen):
    screen.fill(PAPER)
    for x in range(0, WIDTH, 30):
        pygame.draw.line(screen, GRID, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, 30):
        pygame.draw.line(screen, GRID, (0, y), (WIDTH, y))


async def main(max_frames=-1):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.SCALED)
    regular = pygame.font.Font(font_path("JetBrainsMono-Regular.ttf"), 16)
    bold = pygame.font.Font(font_path("JetBrainsMono-Bold.ttf"), 16)
    large = pygame.font.Font(font_path("JetBrainsMono-Bold.ttf"), 34)
    clock = pygame.time.Clock()
    stepper = FixedStepper(STEP)

    x = prev_x = float(LEFT)
    speed = 360.0  # pixels per second
    updates_now = frames_now = ups = fps = frames = 0
    second = 0.0

    running = True
    while running and (max_frames < 0 or frames < max_frames):
        frame_time = clock.tick() / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                running = False

        for _ in range(stepper.advance(frame_time)):
            prev_x = x
            x += speed * STEP
            if x < LEFT or x > RIGHT:
                x = max(float(LEFT), min(x, float(RIGHT)))
                speed = -speed
            updates_now += 1

        second += frame_time
        frames_now += 1
        if second >= 1.0:
            ups, fps = updates_now, frames_now
            updates_now = frames_now = 0
            second -= 1.0

        draw_x = prev_x + (x - prev_x) * stepper.alpha
        backdrop(screen)
        pygame.draw.rect(screen, TEAL, (0, 0, WIDTH, 30))
        screen.blit(bold.render("[ UPDATE LOOP PYTHON ]", True, WHITE), (12, 5))
        right = regular.render("fixed timestep 1/60 s, pygame-ce", True, TEAL_SOFT)
        screen.blit(right, (WIDTH - right.get_width() - 12, 5))

        for left, value, label, color in ((40, ups, "updates per second", TEAL), (300, fps, "frames per second", ORANGE)):
            card(screen, (left, 56, 240, 78), WHITE)
            pygame.draw.rect(screen, color, (left, 56, 6, 78))
            screen.blit(large.render(str(value), True, color), (left + 22, 60))
            screen.blit(regular.render(label, True, MUTED), (left + 22, 104))

        pygame.draw.rect(screen, (196, 197, 201), (LEFT, 300, RIGHT - LEFT + SIZE, 2))
        card(screen, (round(draw_x), 226, SIZE, SIZE), TEAL)

        screen.blit(bold.render("accumulator", True, INK), (40, 372))
        pygame.draw.rect(screen, WHITE, (40, 400, WIDTH - 80, 26))
        pygame.draw.rect(screen, TEAL, (40, 400, round((WIDTH - 80) * stepper.alpha), 26))
        pygame.draw.rect(screen, INK, (40, 400, WIDTH - 80, 26), 1)
        screen.blit(regular.render("leftover time inside one 1/60 s step, used as alpha for the interpolation", True, MUTED), (40, 436))
        pygame.display.flip()
        frames += 1
        await asyncio.sleep(1 / 120)  # about 120 frames per second, and lets the browser breathe under Pyodide

    pygame.quit()


if __name__ == "__main__":
    limit = int(sys.argv[sys.argv.index("--frames") + 1]) if "--frames" in sys.argv else -1
    asyncio.run(main(limit))
