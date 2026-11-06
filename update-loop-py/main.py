"""Fixed timestep update loop with pygame-ce.

Runs natively (`python main.py`) and in the browser through Pyodide.
"""
import asyncio
import sys

import pygame

from stepper import FixedStepper

WIDTH, HEIGHT, SIZE = 640, 360, 40
STEP = 1 / 60


async def main(max_frames=-1):
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    font = pygame.font.Font(None, 20)
    clock = pygame.time.Clock()
    stepper = FixedStepper(STEP)

    x = prev_x = 0.0
    speed = 240.0  # pixels per second
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
            if x < 0 or x > WIDTH - SIZE:
                x = max(0.0, min(x, WIDTH - SIZE))
                speed = -speed
            updates_now += 1

        second += frame_time
        frames_now += 1
        if second >= 1.0:
            ups, fps = updates_now, frames_now
            updates_now = frames_now = 0
            second -= 1.0

        draw_x = prev_x + (x - prev_x) * stepper.alpha
        screen.fill((255, 255, 255))
        pygame.draw.rect(screen, (30, 100, 200), (draw_x, (HEIGHT - SIZE) / 2, SIZE, SIZE))
        screen.blit(font.render(f"updates/s {ups}   frames/s {fps}", True, (40, 40, 40)), (8, 8))
        pygame.display.flip()
        frames += 1
        await asyncio.sleep(1 / 120)  # about 120 frames per second, and lets the browser breathe under Pyodide

    pygame.quit()


if __name__ == "__main__":
    limit = int(sys.argv[sys.argv.index("--frames") + 1]) if "--frames" in sys.argv else -1
    asyncio.run(main(limit))
