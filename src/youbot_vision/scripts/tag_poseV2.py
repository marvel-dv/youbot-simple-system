#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

# ===================== CONFIGURAÇÕES =====================
POSITION_THRESHOLD = 0.005  # metros (5 mm)
TAG_OFFSET_X = 0.00         
# ========================================================

class AprilTagFilterNode:
    def __init__(self):
        rospy.init_node("apriltag_pose_to_base")
        
        # Variáveis de estado
        self.target_tag_id = 1  # ID padrão
        self.filter_active = False  # Flag para saber se filtro foi ativado
        self.last_target_id = 1  # Último ID solicitado
        
        # Buffer TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Publisher
        self.pub_pose = rospy.Publisher(
            "/apriltag/pose_target",
            PoseStamped,
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            "/tag_detections",
            AprilTagDetectionArray,
            self.tag_callback
        )
        
        rospy.Subscriber(
            "/filter",
            Int32,
            self.filter_callback
        )
        
        rospy.loginfo(f"AprilTag node iniciado. ID de busca: {self.target_tag_id}")
        rospy.loginfo("Aguardando mensagens em /tag_detections e /filter...")
    
    def filter_callback(self, msg):
        """
        Callback para receber novo ID da tag alvo
        """
        new_id = int(msg.data)  
        self.target_tag_id = new_id
        self.filter_active = True
        self.last_target_id = new_id
            
        rospy.loginfo(f"Filtro atualizado: monitorando AprilTag ID {self.target_tag_id}")
            
    
    def tag_callback(self, msg):
        """
        Callback para detecções de AprilTag
        """
        if len(msg.detections) == 0:
            return
        
        tag_found = False
        
        for det in msg.detections:
            if not det.id:  # Verifica se a lista de IDs não está vazia
                continue
            
            tag_id = det.id[0]
            
            # Se filtro não foi ativado, usa ID padrão ou último solicitado
            if not self.filter_active:
                target_id = self.last_target_id
            else:
                target_id = self.target_tag_id
            
            # Filtro por ID
            if tag_id != target_id:
                continue
            
            tag_found = True
            frame_cam = det.pose.header.frame_id
            pose_cam = PoseStamped()
            pose_cam.header = det.pose.header
            pose_cam.pose = det.pose.pose.pose
            
            pos = pose_cam.pose.position
            ori = pose_cam.pose.orientation
            
            rospy.loginfo_throttle(1.0, "========== TAG DETECTADA ==========")
            rospy.loginfo_throttle(1.0, f"ID: {tag_id}")
            rospy.loginfo_throttle(1.0, f"Frame da detecção: {frame_cam}")
            
            rospy.loginfo_throttle(1.0,
                "Posição NO FRAME DA CÂMERA [m]: x={:.4f}, y={:.4f}, z={:.4f}".format(
                    pos.x, pos.y, pos.z
                )
            )
            
            try:
                # Transformar para base_footprint
                pose_base = self.tf_buffer.transform(
                    pose_cam,
                    "base_footprint",
                    rospy.Duration(0.5)
                )
                
                pose_base.pose.position.x += TAG_OFFSET_X
                
                rospy.loginfo_throttle(1.0,
                    "Posição NO FRAME base_footprint [m]: x={:.4f}, y={:.4f}, z={:.4f}".format(
                        pose_base.pose.position.x,
                        pose_base.pose.position.y,
                        pose_base.pose.position.z
                    )
                )
                
                # Publicar pose transformada
                self.pub_pose.publish(pose_base)
                
            except (tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(1.0, f"Falha ao transformar pose para base_footprint: {e}")
                return
            
            rospy.loginfo_throttle(1.0, f"{'='*35}")
            
            # Processa apenas a primeira tag que atende ao filtro
            break
        
        if not tag_found and rospy.Time.now().to_sec() % 5 < 0.1:  # Log a cada ~5 segundos
            target_id = self.target_tag_id if self.filter_active else self.last_target_id
            rospy.logwarn(f"Tag ID {target_id} não detectada. Tags disponíveis: {[d.id[0] for d in msg.detections if d.id]}")

def main():
    node = AprilTagFilterNode()
    rospy.spin()

if __name__ == "__main__":
    main()
