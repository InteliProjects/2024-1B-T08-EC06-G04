# Importações de bibliotecas necessárias para o funcionamento do código
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Empty
import threading
import pygame
import cv2
import base64
import numpy as np
from PIL import Image
import io
import queue
import time
import collections

# Inicializa a biblioteca Pygame
pygame.init()

# Configuração da janela de exibição do Pygame
screen = pygame.display.set_mode((1920, 1080))
pygame.display.set_caption("Teleoperação do Robô")

# Carrega e redimensiona imagens de setas para os botões
arrow_up = pygame.image.load('arrow_up.png').convert_alpha()
arrow_down = pygame.image.load('arrow_down.png').convert_alpha()
arrow_left = pygame.image.load('arrow_left.png').convert_alpha()
arrow_right = pygame.image.load('arrow_right.png').convert_alpha()

# Define retângulos para os botões na interface gráfica
button_rects = {
    "kill": pygame.Rect(10, 10, 100, 50),
    "Frente": pygame.Rect(910, 10, 100, 50),
    "Tras": pygame.Rect(910, 70, 100, 50),
    "left": pygame.Rect(800, 40, 100, 50),
    "right": pygame.Rect(1020, 40, 100, 50),
}

# Redimensiona as imagens das setas para se ajustarem aos botões
arrow_up = pygame.transform.scale(arrow_up, (button_rects["Frente"].width, button_rects["Frente"].height))
arrow_down = pygame.transform.scale(arrow_down, (button_rects["Tras"].width, button_rects["Tras"].height))
arrow_left = pygame.transform.scale(arrow_left, (button_rects["left"].width, button_rects["left"].height))
arrow_right = pygame.transform.scale(arrow_right, (button_rects["right"].width, button_rects["right"].height))

# Função para desenhar botões e setas na tela
def draw_buttons():
    for label, rect in button_rects.items():
        pygame.draw.rect(screen, (200, 200, 200), rect, border_radius=10)
    screen.blit(arrow_up, (button_rects["Frente"].topleft))
    screen.blit(arrow_down, (button_rects["Tras"].topleft))
    screen.blit(arrow_left, (button_rects["left"].topleft))
    screen.blit(arrow_right, (button_rects["right"].topleft))

# Cria uma fila para gerenciar atualizações da interface do usuário
ui_queue = queue.Queue(maxsize=10)  # Tamanho máximo pequeno para evitar alta latência

# Classe que controla o robô
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.emergency_client = self.create_client(Empty, 'emergency_stop')  # Novo tópico ROS2 feito para interromper a execução do robô
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.killed = False
        self.safety_distance = 0.35
        self.front_clear = True
        self.back_clear = True

    # Callback do sensor LiDAR
    def lidar_callback(self, msg):
        num_ranges = len(msg.ranges)
        sector_size = num_ranges // 12

        front_left_indices = range(num_ranges - sector_size, num_ranges)
        front_right_indices = range(0, sector_size)
        back_indices = range(5 * sector_size, 7 * sector_size)

        front_ranges = [msg.ranges[i] for i in front_left_indices if 0.01 < msg.ranges[i] < 100.0] + \
                       [msg.ranges[i] for i in front_right_indices if 0.01 < msg.ranges[i] < 100.0]
        back_ranges = [msg.ranges[i] for i in back_indices if 0.01 < msg.ranges[i] < 100.0]

        self.front_clear = not any(r < self.safety_distance for r in front_ranges)
        self.back_clear = not any(r < self.safety_distance for r in back_ranges)

        if not self.front_clear and self.linear_speed > 0:
            self.stop_robot()
        elif not self.back_clear and self.linear_speed < 0:
            self.stop_robot()

    # Movimenta o robô
    def move_robot(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher.publish(msg)
        print(f"Movendo: velocidade linear={self.linear_speed} m/s, velocidade angular={self.angular_speed} rad/s")

    # Para o robô
    def stop_robot(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.move_robot()
        print("Parando o robô.")

    # Aumenta a velocidade linear
    def increase_linear_speed(self):
        if self.front_clear:
            self.linear_speed = 0.1
            self.move_robot()
        else:
            self.stop_robot()

    # Diminui a velocidade linear
    def decrease_linear_speed(self):
        if self.back_clear:
            self.linear_speed = -0.1
            self.move_robot()
        else:
            self.stop_robot()

    # Aumenta a velocidade angular
    def increase_angular_speed(self):
        self.angular_speed = 0.4
        self.move_robot()

    # Diminui a velocidade angular
    def decrease_angular_speed(self):
        self.angular_speed = -0.4
        self.move_robot()

    # Envia um sinal de parada de emergência
    def send_emergency_stop(self):
        if self.emergency_client.wait_for_service(timeout_sec=1.0):
            req = Empty.Request()
            future = self.emergency_client.call_async(req)
            future.add_done_callback(self.emergency_stop_callback)
        else:
            print('Serviço de emergência não disponível, DESLIGUE O ROBÔ')

    # Callback para a parada de emergência
    def emergency_stop_callback(self, future):
        try:
            future.result()
            print('Sinal de parada de emergência enviado com sucesso, processo do robô terminado.')
        except Exception as e:
            print(f'Falha ao chamar o serviço de parada de emergência: {e}, RETIRE A BATERIA DO ROBÔ!!!')

    # Interrupção de emergência
    def kill_switch(self):
        print("Parada de emergência forçada do processo.")
        self.stop_robot()
        self.send_emergency_stop()
        rclpy.shutdown()

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription

    # Callback do ouvinte para processar mensagens recebidas
    def listener_callback(self, msg):
        timestamp, jpg_as_text = msg.data.split('|', 1)
        timestamp = float(timestamp)
        current_time = time.time()
        latency = current_time - timestamp

        jpg_original = base64.b64decode(jpg_as_text)
        jpg_as_np = np.frombuffer(jpg_original, dtype=np.uint8)
        img = cv2.imdecode(jpg_as_np, cv2.IMREAD_COLOR)
        
        if img is not None:
            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_image = Image.fromarray(img_rgb)
            img_bytes = io.BytesIO()
            pil_image.save(img_bytes, format="JPEG")
            img_bytes.seek(0)
            if not ui_queue.full():
                ui_queue.put((img_bytes, latency))
        else:
            self.get_logger().error('Não foi possível decodificar a imagem')

def init_ros_nodes():
    rclpy.init()
    robot_controller = RobotController()
    listener = Listener()

    executor_thread = threading.Thread(target=spin_nodes, args=(robot_controller, listener), daemon=True)
    executor_thread.start()

    return robot_controller, listener

def spin_nodes(robot_controller, listener):
    while rclpy.ok():
        rclpy.spin_once(robot_controller, timeout_sec=0.01)
        rclpy.spin_once(listener, timeout_sec=0.01)

robot_controller, listener = init_ros_nodes()

def main():
    running = True
    clock = pygame.time.Clock()

    # Configuração da fonte
    font = pygame.font.Font(None, 36)
    
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_w:
                    robot_controller.increase_linear_speed()
                elif event.key == pygame.K_s:
                    robot_controller.decrease_linear_speed()
                elif event.key == pygame.K_a:
                    robot_controller.increase_angular_speed()
                elif event.key == pygame.K_d:
                    robot_controller.decrease_angular_speed()
                elif event.key == pygame.K_SPACE:
                    robot_controller.stop_robot()
                elif event.key == pygame.K_q:
                    running = False
                elif event.key == pygame.K_b:
                    robot_controller.kill_switch()
                    running = False
            elif event.type == pygame.KEYUP:
                if event.key in [pygame.K_w, pygame.K_s, pygame.K_a, pygame.K_d]:
                    robot_controller.stop_robot()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_pos = event.pos
                if button_rects["kill"].collidepoint(mouse_pos):
                    robot_controller.kill_switch()
                    running = False
                elif button_rects["Frente"].collidepoint(mouse_pos):
                    robot_controller.increase_linear_speed()
                elif button_rects["Tras"].collidepoint(mouse_pos):
                    robot_controller.decrease_linear_speed()
                elif button_rects["left"].collidepoint(mouse_pos):
                    robot_controller.increase_angular_speed()
                elif button_rects["right"].collidepoint(mouse_pos):
                    robot_controller.decrease_angular_speed()

        # Monitoramento de latência
        latency_values = collections.deque(maxlen=10) 

        if not ui_queue.empty():
            img_bytes, latency = ui_queue.get()
            img_np = np.array(Image.open(img_bytes))
            img_np = np.rot90(img_np, 1)
            img_np = cv2.resize(img_np, (1080, 1920))
            img_surface = pygame.surfarray.make_surface(img_np)
            screen.blit(img_surface, (0, 0))

            latency_ms = latency * 1000
            latency_values.append(latency_ms)
            
            # Exibe texto da latência
            avg_latency = sum(latency_values) / len(latency_values)
            latency_text = font.render(f"Latência: {avg_latency:.2f} ms", True, (255, 255, 255))
            screen.blit(latency_text, (1600, 10))  # Ajuste de posição conforme necessário
        
        draw_buttons()
        pygame.display.flip()
        
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()
