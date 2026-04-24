import rclpy
from rclpy.node import Node
# 移除顶层导入 VideoPacket 和 QoS（改为按需导入）
import av
import cv2
import threading
import queue
from pathlib import Path
import struct
import paho.mqtt.client as mqtt

# 导入 protobuf 生成的类（确保 video_stream_pb2.py 在同目录）
from . import video_stream_pb2


class VideoDecoderNode(Node):
    def __init__(self):
        super().__init__('video_decoder_node')
        
        # ---- 通用参数 ----
        self.declare_parameter('display', True)
        self.declare_parameter('width', 400)
        self.declare_parameter('height', 400)
        self.declare_parameter('display_scale', 2)
        self.declare_parameter('crosshair_offset_x', 0)
        self.declare_parameter('crosshair_offset_y', 0)
        self.declare_parameter('crosshair_width', 2)
        self.declare_parameter('debug_dump_enable', False)
        self.declare_parameter('debug_dump_every_n_frames', 20)
        self.declare_parameter('debug_dump_save_decoder', True)
        self.declare_parameter('debug_dump_dir', 'sniper_debug_imgs')

        # ---- 传输方式参数 ----
        self.declare_parameter('use_mqtt', False)
        self.declare_parameter('mqtt_ip', '192.168.12.1')
        self.declare_parameter('mqtt_port', 3333)
        self.declare_parameter('mqtt_topic', 'CustomByteBlock')
        self.declare_parameter('robot_id', 'decoder1')
        self.declare_parameter('topic', '/video_stream')

        # 读取参数值
        self.display = self.get_parameter('display').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.display_scale = max(1, int(self.get_parameter('display_scale').value))
        self.display_width = self.width * self.display_scale
        self.display_height = self.height * self.display_scale
        self.crosshair_offset_x = int(self.get_parameter('crosshair_offset_x').value)
        self.crosshair_offset_y = int(self.get_parameter('crosshair_offset_y').value)
        self.crosshair_width = max(1, int(self.get_parameter('crosshair_width').value))
        self.debug_dump_enable = bool(self.get_parameter('debug_dump_enable').value)
        self.debug_dump_every_n_frames = max(1, int(self.get_parameter('debug_dump_every_n_frames').value))
        self.debug_dump_save_decoder = bool(self.get_parameter('debug_dump_save_decoder').value)
        self.debug_dump_dir = Path(str(self.get_parameter('debug_dump_dir').value)) / 'decoder'
        self.display_frame_counter = 0

        self.use_mqtt = bool(self.get_parameter('use_mqtt').value)
        self.mqtt_ip = self.get_parameter('mqtt_ip').value
        self.mqtt_port = int(self.get_parameter('mqtt_port').value)
        self.mqtt_topic = self.get_parameter('mqtt_topic').value
        self.robot_id = self.get_parameter('robot_id').value
        self.ros_topic = self.get_parameter('topic').value

        if self.debug_dump_enable and self.debug_dump_save_decoder:
            self.debug_dump_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(
                f'Debug dump enabled: every {self.debug_dump_every_n_frames} frames -> {self.debug_dump_dir}'
            )
        elif self.debug_dump_enable:
            self.get_logger().warn('debug_dump_enable=true but debug_dump_save_decoder=false')

        # ---- 解码器状态 ----
        self.codec = None
        self._reset_decoder(log=False, reason='startup')
        self.frame_count = 0
        self.packet_count = 0
        self.parsed_packet_count = 0
        self.gap_count = 0
        self.last_seq = None

        # ---- 显示队列 ----
        if self.display:
            self.frame_queue = queue.Queue(maxsize=3)
            self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
            self.display_thread.start()

        # ---- 根据传输方式启动接收 ----
        if self.use_mqtt:
            self._init_mqtt()
        else:
            # 仅当使用 ROS2 时才导入消息类型和 QoS
            from doorlock_sniper.msg import VideoPacket
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=3000
            )
            self.subscription = self.create_subscription(
                VideoPacket,
                self.ros_topic,
                self._ros2_packet_callback,
                qos
            )
            self.get_logger().info(f'Decoder started: subscribing to ROS2 topic {self.ros_topic}')

    # ==================== 解码器重置 ====================
    def _reset_decoder(self, *, log: bool = True, reason: str = ''):
        self.codec = av.CodecContext.create('h264', 'r')
        self.codec.thread_type = 'FRAME'
        self.codec.flags |= av.codec.context.Flags.LOW_DELAY
        if log:
            self.get_logger().warn(f'Reset decoder ({reason})')

    # ==================== 处理解码帧 ====================
    def _handle_decoded_frame(self, frame):
        if frame is None or frame.width == 0 or frame.height == 0:
            return

        img = frame.to_ndarray(format='bgr24')
        if img is None or img.size == 0:
            return

        self.frame_count += 1
        if self.display:
            try:
                self.frame_queue.put_nowait(img)
            except queue.Full:
                pass
        elif self.frame_count % 60 == 0:
            self.get_logger().info(f'Decoded {self.frame_count} frames')

    # ==================== 通用数据注入（ROS2/MQTT共用） ====================
    def _process_video_chunk(self, seq: int, ts_ns: int, payload_282: bytes):
        if len(payload_282) != 282:
            self.get_logger().warn(f'Invalid payload size: {len(payload_282)} (expected 282)')
            return

        self.packet_count += 1
        if self.last_seq is not None and seq != self.last_seq + 1:
            self.gap_count += 1
            self.get_logger().warn(
                f'Gap detected: {self.last_seq} -> {seq}, reset decoder')
            self._reset_decoder(reason='sequence gap')
        self.last_seq = seq

        try:
            parsed_packets = self.codec.parse(payload_282)
            self.parsed_packet_count += len(parsed_packets)
            for packet in parsed_packets:
                for frame in self.codec.decode(packet):
                    self._handle_decoded_frame(frame)
        except av.AVError as e:
            self.get_logger().debug(f'Decode error: {e!s}')

        if self.packet_count % 600 == 0:
            self.get_logger().info(
                f'Rx packets={self.packet_count} parsed_h264={self.parsed_packet_count} '
                f'decoded_frames={self.frame_count} gaps={self.gap_count}')

    # ==================== ROS2 回调 ====================
    def _ros2_packet_callback(self, msg):
        self._process_video_chunk(msg.sequence_id, msg.timestamp_ns, bytes(msg.data))

    # ==================== MQTT 初始化 ====================
    def _init_mqtt(self):
        self.mqtt_client = mqtt.Client(client_id=self.robot_id, protocol=mqtt.MQTTv311)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_message = self._on_mqtt_message
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect

        try:
            self.mqtt_client.connect(self.mqtt_ip, self.mqtt_port, keepalive=20)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'MQTT client connecting to {self.mqtt_ip}:{self.mqtt_port} ...')
        except Exception as e:
            self.get_logger().error(f'MQTT connection failed: {e}')

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT connected')
            client.subscribe(self.mqtt_topic, qos=0)
            self.get_logger().info(f'Subscribed to {self.mqtt_topic}')
        else:
            self.get_logger().error(f'MQTT connection failed, rc={rc}')

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self.get_logger().warn(f'MQTT disconnected (rc={rc}), auto-reconnect enabled')

    def _on_mqtt_message(self, client, userdata, msg):
        """使用 Protobuf 解析 CustomByteBlock 消息"""
        try:
            block = video_stream_pb2.CustomByteBlock()
            block.ParseFromString(msg.payload)
            raw_data = block.data  # 300 字节

            if len(raw_data) != 300:
                self.get_logger().warn(f'Unexpected data size: {len(raw_data)} (expected 300)')
                return

            # 解析头部 (18 字节) + H.264 负载 (282 字节)
            header = raw_data[:18]
            seq, ts_ns, payload_size = struct.unpack('<Q Q H', header)

            if payload_size != 282:
                self.get_logger().warn(f'Wrong payload size in header: {payload_size}')
                return

            payload = raw_data[18:18+282]
            self._process_video_chunk(seq, ts_ns, payload)

        except Exception as e:
            self.get_logger().error(f'MQTT message parse error: {e}')

    def _sharpen_center(self, img):
        """
        对图像中心区域做 Unsharp Mask 锐化
        """
        h, w = img.shape[:2]
        # 中心区域大小（与编码端 center_clear_size 对应，但这里是显示尺寸）
        # 由于显示端可能做了放大，我们以图像百分比定义中心区域
        center_w = int(w * 0.4)       # 宽度占 40%
        center_h = int(h * 0.4)
        x0 = max(0, (w - center_w) // 2)
        y0 = max(0, (h - center_h) // 2)
        x1 = min(w, x0 + center_w)
        y1 = min(h, y0 + center_h)

        # 提取中心 ROI
        roi = img[y0:y1, x0:x1]
        # Unsharp Mask
        blurred = cv2.GaussianBlur(roi, (0, 0), 0.8)   # 小 sigma 模糊
        sharpened = cv2.addWeighted(roi, 1.5, blurred, -0.5, 0)
        # 写回原图
        img[y0:y1, x0:x1] = sharpened
        return img
    # ==================== 显示线程 ====================
    def _display_loop(self):
        cv2.namedWindow('Doorlock Decoder', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Doorlock Decoder', self.display_width, self.display_height)
        
        while rclpy.ok():
            try:
                img = self.frame_queue.get(timeout=0.05)
                if img is None:
                    break
                if img.size > 0:
                    img_disp = cv2.resize(
                        img,
                        (self.display_width, self.display_height),
                        interpolation=cv2.INTER_NEAREST
                    )
                    img_disp = self._sharpen_center(img_disp)
                    self._draw_overlay(img_disp)
                    cv2.imshow('Doorlock Decoder', img_disp)
                    if self.debug_dump_enable and self.debug_dump_save_decoder:
                        self.display_frame_counter += 1
                        if self.display_frame_counter % self.debug_dump_every_n_frames == 0:
                            frame_id = f'{self.display_frame_counter:08d}'
                            out_path = self.debug_dump_dir / f'decoder_{frame_id}.png'
                            cv2.imwrite(str(out_path), img_disp)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.get_logger().info('User quit')
                        rclpy.shutdown()
                        break
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Display error: {e}')
                break
        
        cv2.destroyAllWindows()

    def _draw_overlay(self, img):
        h, w = img.shape[:2]

        # 准心
        cx = max(0, min(w - 1, w // 2 + self.crosshair_offset_x))
        cy = max(0, min(h - 1, h // 2 + self.crosshair_offset_y))

        crosshair_color = (230, 190, 235)  # BGR 淡紫
        cv2.line(img, (0, cy), (w - 1, cy), crosshair_color, self.crosshair_width, cv2.LINE_AA)
        cv2.line(img, (cx, 0), (cx, h - 1), crosshair_color, self.crosshair_width, cv2.LINE_AA)

        center_color = (170, 255, 170)  # BGR 淡绿
        center = (w // 2, h // 2)
        cv2.circle(img, center, 24, center_color, 1, cv2.LINE_AA)

    # ==================== 清理 ====================
    def destroy_node(self):
        if self.use_mqtt:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        else:
            pass  # ROS2 订阅自动销毁

        if self.display:
            try:
                self.frame_queue.put_nowait(None)
            except queue.Full:
                pass
            self.display_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoDecoderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()