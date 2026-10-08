"""Small live benchmark window driven by the runner's JSON String status."""

import json

import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class BenchmarkHud(Node):
    def __init__(self):
        super().__init__("kcity_benchmark_hud")
        self.status = {"status": "waiting", "suite": "?"}
        self.create_subscription(String, "/kcity/benchmark/status", self.update, 10)
        self.get_logger().info("Live HUD source: /kcity/benchmark/status")

    def update(self, msg):
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"Invalid benchmark status JSON: {exc}")

    def run(self):
        pygame.init()
        pygame.display.set_caption("K-City Benchmark HUD")
        window = pygame.display.set_mode((500, 480))
        font = pygame.font.Font(None, 25)
        clock = pygame.time.Clock()
        running = True
        try:
            while running and rclpy.ok():
                clock.tick(10)
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                rclpy.spin_once(self, timeout_sec=0.0)
                s = self.status
                latest = s.get("latest_mission_result") or {}
                rows = [
                    f"Suite: {s.get('suite')}    State: {s.get('status')}",
                    f"Lap (CARLA sim): {s.get('lap_time_sec')} s",
                    f"Completion: {s.get('completion')} %",
                    f"Mission penalty: {s.get('mission_penalty_sec', 0.0)} s",
                    f"Lane penalty: {s.get('lane_penalty_sec', 0.0)} s",
                    f"Total penalty: {s.get('total_penalty_sec', s.get('accumulated_penalty_sec'))} s",
                    f"Current mission: {s.get('current_mission') or '-'}",
                    f"Latest mission: {latest.get('id', '-')} {latest.get('status', '')}",
                ]
                if s.get("emergency_stop_center_relocation_required"):
                    rows.append("COURSE OUT: emergency stop / center relocation required")
                if s.get("status") == "finished":
                    scores = s.get("scores") or {}
                    rows.extend([
                        f"Final record: {s.get('final_record_sec')} s",
                        f"Completion: {scores.get('completion')}  Efficiency: {scores.get('efficiency')}",
                        f"Safety: {scores.get('safety')}  Mission: {scores.get('mission')}",
                        f"Comfort: {scores.get('comfort')}  Quality: {scores.get('quality')}",
                        f"Total score: {scores.get('total')}",
                    ])
                else:
                    rows.append("Waiting for FINISH exit-edge crossing...")
                window.fill((17, 25, 33))
                for i, row in enumerate(rows):
                    window.blit(font.render(row, True, (231, 236, 241)), (15, 16 + i * 30))
                pygame.display.flip()
        finally:
            pygame.quit()


def main(args=None):
    rclpy.init(args=args)
    node = BenchmarkHud()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
