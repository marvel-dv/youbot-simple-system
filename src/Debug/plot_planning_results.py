#!/usr/bin/env python3
import csv
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

"""
python3 plot_planning_results.py
"""

# ==============================
# Carregar CSV
# ==============================
filename = "planning_test01.csv"

rep = []
pose_id = []

x_des = []
y_des = []
z_des = []

err_x = []
err_y = []
err_z = []
err_norm = []
err_ang_deg = []




with open(filename, "r") as f:
    reader = csv.DictReader(f)
    for row in reader:
        rep.append(int(row["rep"]))
        pose_id.append(int(row["pose_id"]))

        x_des.append(float(row["x_des"]))
        y_des.append(float(row["y_des"]))
        z_des.append(float(row["z_des"]))

        err_x.append(float(row["err_x"]))
        err_y.append(float(row["err_y"]))
        err_z.append(float(row["err_z"]))
        err_norm.append(float(row["err_norm"]))
        err_ang_deg.append(float(row["err_ang_deg"]))

rep = np.array(rep)
pose_id = np.array(pose_id)

x_des = np.array(x_des)
y_des = np.array(y_des)
z_des = np.array(z_des)

err_x = np.array(err_x)
err_y = np.array(err_y)
err_z = np.array(err_z)
err_norm = np.array(err_norm)
err_ang_deg = np.array(err_ang_deg)

print(f"Dados carregados: {len(err_norm)} amostras")

# ==============================
# Estatísticas
# ==============================
def stats(name, data, unit=""):
    data = np.asarray(data)

    print(f"\n{name}")
    print(f"  Média        : {np.mean(data):.4f} {unit}")
    print(f"  Desvio padrão: {np.std(data):.4f} {unit}")
    print(f"  Máximo       : {np.max(np.abs(data)):.4f} {unit}")
    print(f"  Mínimo       : {np.min(np.abs(data)):.4f} {unit}")

#\==============================
#Métricas
#==============================

stats("Erro em X", err_x, "m")
stats("Erro em Y", err_y, "m")
stats("Erro em Z", err_z, "m")
stats("Erro de posição (norma)", err_norm, "m")
stats("Erro de orientação", err_ang_deg, "deg")


# ==============================
# 1- Boxplot – erro de posição
# ==============================
plt.figure()
plt.boxplot(err_norm, showmeans=True)
plt.ylabel("Erro de posição [m]")
plt.title("Distribuição do erro de posição")
plt.grid(True)
plt.tight_layout()

# ==============================
# 2- Boxplot – erro de orientação
# ==============================
plt.figure()
plt.boxplot(err_ang_deg, showmeans=True)
plt.ylabel("Erro angular [graus]")
plt.title("Distribuição do erro de orientação")
plt.grid(True)
plt.tight_layout()

# ============================== 
# 3-Dispersão X–Y do erro 
# ============================== 
plt.figure() 
plt.scatter(err_x, err_y, s=20) 
plt.axhline(0) 
plt.axvline(0) 
plt.xlabel("Erro em X [m]") 
plt.ylabel("Erro em Y [m]") 
plt.title("Dispersão do erro planar X-Y") 
plt.axis("equal") 
plt.grid(True) 
plt.tight_layout()



# ==============================
# 5- Erro ao longo das execuções
# ==============================
plt.figure()
plt.plot(err_norm, marker="o", linestyle="")
plt.xlabel("Execução")
plt.ylabel("Erro de posição [m]")
plt.title("Erro de posição ao longo das execuções")
plt.grid(True)
plt.tight_layout()

# ==============================
# 6- ERRO MÉDIO POR POSE 
# ==============================
err_pose = defaultdict(list)
ang_pose = defaultdict(list)

for pid, e, a in zip(pose_id, err_norm, err_ang_deg):
    err_pose[pid].append(e)
    ang_pose[pid].append(a)

pose_ids = sorted(err_pose.keys())
pose_ids = sorted(ang_pose.keys())

mean_err = [np.mean(err_pose[p]) for p in pose_ids]
std_err = [np.std(err_pose[p]) for p in pose_ids]

mean_ang = [np.mean(ang_pose[p]) for p in pose_ids]
std_ang = [np.std(ang_pose[p]) for p in pose_ids]

plt.figure()
plt.errorbar(
    pose_ids,
    mean_err,
    yerr=std_err,
    fmt="o",
    capsize=5
)
plt.xlabel("ID da pose")
plt.ylabel("Erro médio de posição [m]")
plt.title("Erro médio de posição por pose")
plt.grid(True)
plt.tight_layout()

#================================
plt.figure()
plt.errorbar(
    pose_ids,
    mean_ang,
    yerr=std_ang,
    fmt="o",
    capsize=5
)
plt.xlabel("ID da pose")
plt.ylabel("Erro médio de orientação [°]")
plt.title("Erro médio de orientação por pose")
plt.grid(True)
plt.tight_layout()

print("\nDesvio padrão por pose:")
print("Pose | Std posição [m] | Std orientação [deg]")
print("---------------------------------------------")

for pid, s_pos, s_ang in zip(pose_ids, std_err, std_ang):
    print(f"{pid:>4} | {s_pos:>14.6f} | {s_ang:>19.4f}")

# ==============================
# ADICIONE ESTA SEÇÃO: ERRO MÉDIO POR POSE
# ==============================
print("\nErro médio por pose:")
print("Pose | Média posição [m] | Média orientação [deg]")
print("--------------------------------------------------")

for pid, m_pos, m_ang in zip(pose_ids, mean_err, mean_ang):
    print(f"{pid:>4} | {m_pos:>17.6f} | {m_ang:>22.4f}")

# Se quiser também mostrar a média geral de todas as poses
print("\nResumo geral de todas as poses:")
print(f"Média global de posição: {np.mean(err_norm):.6f} m")
print(f"Média global de orientação: {np.mean(err_ang_deg):.4f} °")
# ==============================
# 7- ERRO POSIÇÃO × ERRO ORIENTAÇÃO
# ==============================
plt.figure()
plt.scatter(err_norm, err_ang_deg, s=30)
plt.xlabel("Erro de posição [m]")
plt.ylabel("Erro de orientação [graus]")
plt.title("Correlação entre erro de posição e orientação")
plt.grid(True)
plt.tight_layout()

# ==============================
# 8- ERRO × DISTÂNCIA AO ROBÔ 
# ==============================
dist = np.sqrt(x_des**2 + y_des**2 + z_des**2)

plt.figure()
plt.scatter(dist, err_norm, s=30)
plt.xlabel("Distância à base [m]")
plt.ylabel("Erro de posição [m]")
plt.title("Erro de posição em função da distância ao robô")
plt.grid(True)
plt.tight_layout()


# ==============================
# Mostrar tudo
# ==============================
plt.show()


