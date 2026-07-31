import numpy as np

d = 0.031
k_m = 2.4 * 10**(-6)
k_f = 4 * 10**(-5)

M = np.zeros((4, 4))
M[0, :] = 1
M[1] = [-d, -d, d, d]
M[2] = [-d, d, d, -d]
M[3] = [-k_m/k_f, k_m/k_f, -k_m/k_f, k_m/k_f]

Minv = np.linalg.inv(M)

def allocate(F, taux, tauy, tauz):
    #[F1,F2,F3,F4] = Minv @ [F,τx,τy,τz]

    #each row of F_i is F1, F2, F3, F4
    F_i = Minv @ np.array([F, taux, tauy, tauz])

    w_i = np.sqrt(np.clip(F_i, 0, None) /k_f)
    w_i = np.clip(w_i, 0, 600)

    #each row of w_i is w1, w2, w3, w4
    return w_i