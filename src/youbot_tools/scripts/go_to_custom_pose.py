#!/usr/bin/env python3
import rospy
from tkinter import *
from youbot_tools.srv import GoToPoseMoveIt
from gazebo_msgs.srv import GetLinkState
import tf.transformations as tftr
import threading
from geometry_msgs.msg import PoseStamped


class GoToPoseCustomGUI:

    def __init__(self):

        # ---- CRIA A GUI PRIMEIRO ----
        self.root = Tk()
        self.root.title("YouBot - Pose Manual (Quaternion)")
        self.root.geometry("420x540")
        self.root.resizable(False, False)

        Label(
            self.root, text="Pose Manual (XYZ + Quaternion)",
            font=("Arial", 16)
        ).pack(pady=10)

        frame = Frame(self.root)
        frame.pack()

        # Campos de entrada
        self.entries = {}
        labels = ["X", "Y", "Z", "Qx", "Qy", "Qz", "Qw"]
        default_vals = ["0.3", "0.0", "0.15", "0", "1", "0", "0"]

        for i, (lab, val) in enumerate(zip(labels, default_vals)):
            Label(frame, text=lab, font=("Arial", 12)).grid(row=i, column=0, pady=5)
            entry = Entry(frame, width=25)
            entry.insert(0, val)
            entry.grid(row=i, column=1, pady=5)
            self.entries[lab] = entry

        # Botões desativados até o ROS conectar
        self.btn_go = Button(self.root, text="IR PARA POSE", width=30, state=DISABLED,
                             command=self.go_to_pose)
        self.btn_go.pack(pady=15)

        self.btn_block_gz = Button(self.root, text="PEGAR POSE DO BLOCO (GAZEBO)", width=30,
                                   state=DISABLED, command=self.get_block_pose_gz)
        self.btn_block_gz.pack(pady=10)
        
        self.btn_block_at = Button(self.root, text="PEGAR POSE DO BLOCO (Apriltag)", width=30,
                                   state=DISABLED, command=self.get_block_pose_at)
        self.btn_block_at.pack(pady=10)

        self.status_label = Label(self.root, text="Inicializando ROS...", fg="blue")
        self.status_label.pack(pady=20)
        
        self.pose_target = None

        # Inicia ROS em thread paralela
        threading.Thread(target=self.init_ros, daemon=True).start()

        self.root.mainloop()

    # ----------------------------------------------------------------------
    def init_ros(self):
        rospy.init_node("go_to_pose_custom_gui", anonymous=True, disable_signals=True)

        # Serviço MoveIt
        rospy.wait_for_service("/youbot/go_to_pose_moveit")
        self.go_to_pose_srv = rospy.ServiceProxy("/youbot/go_to_pose_moveit", GoToPoseMoveIt)
        
        # Subscriber pose_target
        rospy.Subscriber("/apriltag/pose_target", PoseStamped, self.pose_callback)

        # Serviço de pose relativa do Gazebo
        rospy.wait_for_service("/gazebo/get_link_state")
        self.get_link_state = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)

        # Habilita botões quando estiver tudo pronto
        self.root.after(0, lambda: self.status_label.config(text="ROS conectado!", fg="green"))
        self.root.after(0, lambda: self.btn_go.config(state=NORMAL))
        self.root.after(0, lambda: self.btn_block_gz.config(state=NORMAL))
        self.root.after(0, lambda: self.btn_block_at.config(state=NORMAL))

    # ----------------------------------------------------------------------
    def go_to_pose(self):
        try:
            # Atualiza status
            self.status_label.config(text="Indo para pose...", fg="orange")
            self.root.update_idletasks()

            x  = float(self.entries["X"].get())
            y  = float(self.entries["Y"].get())
            z  = float(self.entries["Z"].get())
            qx = float(self.entries["Qx"].get())
            qy = float(self.entries["Qy"].get())
            qz = float(self.entries["Qz"].get())
            qw = float(self.entries["Qw"].get())

            # Quaternion -> RPY
            roll, pitch, yaw = tftr.euler_from_quaternion([qx, qy, qz, qw])

            resp = self.go_to_pose_srv(x, y, z, roll, pitch, yaw)

            color = "green" if resp.success else "red"
            self.status_label.config(text=resp.message, fg=color)

        except Exception as e:
            self.status_label.config(text=f"Erro: {e}", fg="red")

    # ----------------------------------------------------------------------
    def get_block_pose_gz(self):
        """
        Pega pose do bloco RELATIVA ao robô diretamente do Gazebo
        usando get_link_state
        """
        try:
            # Nome do link do bloco
            block_link = "block_test::block_link"

            # Nome do link base do youbot no Gazebo 
            robot_base = "youbot::base_footprint"

            resp = self.get_link_state(link_name=block_link, reference_frame=robot_base)

            if not resp.success:
                self.status_label.config(text="Falha ao obter pose!", fg="red")
                return

            pos = resp.link_state.pose.position
            ori = resp.link_state.pose.orientation
            
            n = 4
            vals = {
                "X": round(pos.x, n),
                "Y": round(pos.y, n),
                "Z": round(pos.z, n),
                "Qx": 0, 
                "Qy": 1,
                "Qz": 0,
                "Qw": 0,
            }

            for k, v in vals.items():
                self.entries[k].delete(0, END)
                self.entries[k].insert(0, str(v))

            self.status_label.config(
                text="Pose do bloco (Gazebo) carregada!",
                fg="green"
            )

        except Exception as e:
            self.status_label.config(text=f"Erro: {e}", fg="red")

    # ----------------------------------------------------------------------
    def get_block_pose_at(self):
        """
        Pega pose do bloco RELATIVA ao robô,
        publicada no tópico /apriltag/pose_target
        """
        try:
            if self.pose_target is None:
                self.status_label.config(
                    text="Nenhuma pose Apriltag recebida ainda!",
                    fg="red"
                )
                return
            
            pos = self.pose_target.pose.position
            ori = self.pose_target.pose.orientation
            
            n = 4
            vals = {
                "X": round(pos.x, n),
                "Y": round(pos.y, n),
                "Z": round(pos.z, n),
                "Qx": 0, 
                "Qy": 1,
                "Qz": 0,
                "Qw": 0,
            }
            #quaternion rotacionado para pegar bloco 
            """
            "Qx": 0.722, 
            "Qy": -0.7,
            "Qz": 0,
            "Qw": 0,
            """
            
            for k, v in vals.items():
                self.entries[k].delete(0, END)
                self.entries[k].insert(0, str(v))
                
            self.status_label.config(
                text="Pose do bloco (Apriltag) carregada!",
                fg="green"
            )

        except Exception as e:
            self.status_label.config(text=f"Erro: {e}", fg="red")

    # ----------------------------------------------------------------------
    def pose_callback(self, msg):
        """
        Callback que recebe a pose do apriltag já em base_link
        """
        self.pose_target = msg

# ----------------------------------------------------------------------
if __name__ == "__main__":
    GoToPoseCustomGUI()

