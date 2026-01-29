#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

##python3 plot_detection_results.py

# ==============================
# Carregar dados
# ==============================
df = pd.read_csv("detection_test(perto).csv")

ex = df["err_x"].values
ey = df["err_y"].values
ez = df["err_z"].values
en = df["err_norm"].values

# ==============================
# Métricas
# ==============================
def stats(name, data):
    print(f"\n{name}")
    print(f"  Média        : {np.mean(data):.4f} m")
    print(f"  Desvio padrão: {np.std(data):.4f} m")
    print(f"  Máximo       : {np.max(np.abs(data)):.4f} m")
    print(f"  Minimo       : {np.min(np.abs(data)):.4f} m")

stats("Erro X", ex)
stats("Erro Y", ey)
stats("Erro Z", ez)
stats("Erro Norma", en)

# ==============================
# Histogramas
# ==============================
plt.figure()
plt.hist(ex, bins=30)
plt.title("Histograma do erro em X")
plt.xlabel("Erro [m]")
plt.ylabel("Frequência")

plt.figure()
plt.hist(ey, bins=30)
plt.title("Histograma do erro em Y")
plt.xlabel("Erro [m]")
plt.ylabel("Frequência")

#plt.figure()
#plt.hist(ez, bins=30)
#plt.title("Histograma do erro em Z")
#plt.xlabel("Erro [m]")
#plt.ylabel("Frequência")


plt.figure()
plt.hist(en, bins=30)
plt.title("Histograma do erro Euclidiano")
plt.xlabel("Erro [m]")
plt.ylabel("Frequência")

# ==============================
# Erro vs distância
# ==============================
dist = np.sqrt(
    df["x_gt"]**2 +
    df["y_gt"]**2 
)

plt.figure()
plt.scatter(dist, en)
plt.xlabel("Distância da base [m]")
plt.ylabel("Erro Euclidiano [m]")
plt.title("Erro por distância detectada")
plt.grid(True)

# ==============================
# Mapa espacial do erro (X vs Y)
# ==============================
plt.figure(figsize=(8, 6))

norm = plt.Normalize(en.min(), en.max())
sc = plt.scatter(
    df["x_gt"],
    df["y_gt"],
    c=en,
    cmap="RdYlGn_r",
    norm=norm
)

plt.colorbar(sc, label="Erro Euclidiano [m]")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Mapa espacial do erro da AprilTag")

plt.axis("equal")
plt.grid(True)






plt.show()

