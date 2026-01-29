#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# python3 plot_approach_results.py

# ==============================
# Carregar dados
# ==============================
df = pd.read_csv("approach_test.csv")

d_goal = df["d_goal"].values
err_tag = df["err_tag_norm"].values
err_app = df["err_approach"].values
dist_final = df["dist_final"].values
x_gt = df["x_gt"].values
y_gt = df["y_gt"].values

# ==============================
# Estatísticas
# ==============================
def stats(name, data):
    print(f"\n{name}")
    print(f"  Média        : {np.mean(data):.4f} m")
    print(f"  Desvio padrão: {np.std(data):.4f} m")
    print(f"  Máximo       : {np.max(np.abs(data)):.4f} m")
    print(f"  Mínimo       : {np.min(np.abs(data)):.4f} m")

stats("Erro percepção (AprilTag)", err_tag)
stats("Erro de approach", err_app)

# ==============================
# Histograma — erro de percepção
# ==============================
plt.figure()
plt.boxplot(err_app, showmeans=True)
plt.ylabel("Erro da aproximação [m]")
plt.title("Distribuição do erro ")
plt.grid(True)
plt.tight_layout()



# ==============================
# 1-Histograma — erro de approach
# ==============================
plt.figure()
plt.hist(err_app, bins=20)
plt.xlabel("Erro de distância [m]")
plt.ylabel("Frequência")
plt.title("Histograma do erro de aproximação")
plt.grid(True)


# ============================== 
# 2-Erro de approach vs distância final 
# ============================== 
plt.figure() 
plt.scatter(dist_final, np.abs(err_app)) 
plt.xlabel("Distância final ao bloco [m]") 
plt.ylabel("Erro de aproximação [m]") 
plt.title("Erro de aproximação vs distância final(n usar)") 
plt.grid(True)


# ==============================
# 3-Erro de approach vs erro de percepção
# ==============================
plt.figure()
plt.scatter(np.abs(err_tag), np.abs(err_app))
plt.xlabel("Erro de percepção [m]")
plt.ylabel("Erro de aproximação [m]")
plt.title("Influência da percepção na aproximação")
plt.grid(True)

# ==============================
# 4-Módulo do Erro de approach × d_goal 
# ==============================
plt.figure()
plt.scatter(d_goal, np.abs(err_app))
plt.xlabel("Distância objetivo [m]")
plt.ylabel("Erro de aproximação [m]")
plt.title("Erro de aproximação em função da distância objetivo")
plt.grid(True)

# ==============================
# 5-Erro médio ± desvio por d_goal 
# ==============================
unique_d = np.unique(d_goal)
mean_err = []
std_err = []

for d in unique_d:
    e = np.abs(err_app[d_goal == d])
    mean_err.append(np.mean(e))
    std_err.append(np.std(e))

mean_err = np.array(mean_err)
std_err = np.array(std_err)

plt.figure()
plt.errorbar(
    unique_d,
    mean_err,
    yerr=std_err,
    fmt="o-",
    capsize=5
)
plt.xlabel("Distância desejada [m]")
plt.ylabel("Erro médio de aproximação [m]")
plt.title("Erro médio de aproximação por distância objetivo")
plt.grid(True)

# ==============================
# 6-Erro × d_goal
# ==============================
plt.figure()
plt.scatter(d_goal, err_app)
plt.xlabel("Distância desejada [m]")
plt.ylabel("Erro de aproximação [m]")
plt.title("Erro de aproximação vs distância desejada(n usar)")
plt.grid(True)


# ==============================
# 7-Módulo do Erro de detecção × distancia real
# ==============================
plt.figure()
plt.scatter(dist_final, np.abs(err_tag))
plt.xlabel("Distância [m]")
plt.ylabel("|Erro| de detecção [m]")
plt.title("Erro de detecção em função da distância")
plt.grid(True)



plt.show()


