#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State


class DroneActivator(Node):
    def __init__(self):
        super().__init__('drone_activator')

        # Parâmetro: nome do próximo drone (ex: 'uav2' quando este é uav1)
        self.declare_parameter('next_uav_name', '')
        self.next_uav_name = self.get_parameter('next_uav_name').get_parameter_value().string_value.strip()

        self.current_state = State()
        self.next_state = State() if self.next_uav_name else None
        self.next_activated = not bool(self.next_uav_name)

        self.state_sub = self.create_subscription(
            State, '/uav1/mavros/state', self.state_callback, 10)

        if self.next_uav_name:
            next_topic = f'/{self.next_uav_name}/mavros/state'
            self.next_state_sub = self.create_subscription(
                State, next_topic, self.next_state_callback, 10)
            self.get_logger().info(f"Este drone vai esperar ativação de: {self.next_uav_name}")
        else:
            self.get_logger().info("Este é o último drone → finaliza assim que ativar")

        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/uav1/mavros/setpoint_position/local', 10)

        self.mode_client = self.create_client(SetMode, '/uav1/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/uav1/mavros/cmd/arming')

        # Setpoint fixo: z = 0.0 → drone NÃO sobe
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'local_origin'
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        self.pose.pose.orientation.w = 1.0

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

        self.get_logger().info("Iniciando ativação: publicando setpoint em z=1.0 (hover mínimo 1m)...")

        self.offboard_requested = False
        self.armed_requested = False
        self.offboard_future = None
        self.arming_future = None

        self.start_time = self.get_clock().now()
        self.done = False

    def state_callback(self, msg):
        self.current_state = msg

    def next_state_callback(self, msg):
        self.next_state = msg
        if msg.armed and msg.mode == "OFFBOARD":
            if not self.next_activated:
                self.next_activated = True
                self.get_logger().info(f"Drone {self.next_uav_name} ativado → posso finalizar")

    def timer_callback(self):
        if self.done:
            return

        now = self.get_clock().now()
        self.pose.header.stamp = now.to_msg()
        self.setpoint_pub.publish(self.pose)

        if not self.offboard_requested and (now - self.start_time) > Duration(seconds=5.0):
            if self.current_state.mode != "OFFBOARD":
                if self.offboard_future is None:
                    self.request_offboard_async()
            else:
                self.offboard_requested = True
                self.get_logger().info("OFFBOARD já está ativo")

        if self.offboard_requested and not self.armed_requested:
            if not self.current_state.armed:
                if self.arming_future is None:
                    self.request_arm_async()
            else:
                self.armed_requested = True
                self.get_logger().info("Este drone armado → verificando próximo")

        if self.armed_requested and self.next_activated:
            self.finish_and_shutdown()

    def request_offboard_async(self):
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Serviço set_mode ainda não disponível")
            return

        req = SetMode.Request()
        req.custom_mode = 'OFFBOARD'

        self.offboard_future = self.mode_client.call_async(req)
        self.offboard_future.add_done_callback(self.offboard_done)
        self.get_logger().info("Solicitando OFFBOARD...")

    def offboard_done(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("OFFBOARD ativado com sucesso!")
                self.offboard_requested = True
            else:
                self.get_logger().error("Falha ao solicitar OFFBOARD")
        except Exception as e:
            self.get_logger().error(f"Erro na chamada OFFBOARD: {str(e)}")
        finally:
            self.offboard_future = None

    def request_arm_async(self):
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Serviço arming ainda não disponível")
            return

        req = CommandBool.Request()
        req.value = True

        self.arming_future = self.arming_client.call_async(req)
        self.arming_future.add_done_callback(self.arming_done)
        self.get_logger().info("Solicitando arm...")

    def arming_done(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Drone armado e em OFFBOARD. MPC assume o controle em 2 s")
                self.armed_requested = True
            else:
                self.get_logger().warn("Falha ao armar")
        except Exception as e:
            self.get_logger().error(f"Erro na chamada arm: {str(e)}")
        finally:
            self.arming_future = None

    def finish_and_shutdown(self):
        if self.done:
            return
        self.done = True
        self.get_logger().info("Ativação completa.")

        self.create_timer(1.0, self._really_shutdown)

    def _really_shutdown(self):
        self.destroy_node()
        rclpy.shutdown()
        self.get_logger().info("Node destruído e rclpy.shutdown() chamado.")


def main(args=None):
    rclpy.init(args=args)
    node = DroneActivator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()