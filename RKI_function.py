from ulab import numpy as np

#The following code is based of of the explanation code given during the exercises A,B,C,D,E of the RKI lecutres
#given by dr. C. Gabellieri PhD.

def tilde(T):
    """
    Make from the twist a matrix instead of vector
    """
    T_tilde = np.zeros((3, 3))
    T_tilde[0, 1] = -T[0]
    T_tilde[1, 0] = T[0]
    T_tilde[:2, 2] = T[1:]

    return T_tilde


def expT(multi):
    """
    Calculate twist exponential of multiplication between tilde Twist and angle q
    """
    Hq = np.eye(3)

    omega = multi[1, 0]
    vx, vy = multi[:2, 2]
    if abs(omega) < (abs(vx) + abs(vy)) * 0.01 or abs(omega) < 1e-5:
        # pure translation
        Hq[0, 2] = vx
        Hq[1, 2] = vy
    else:
        # rotation and translation
        rx = -vy / omega
        ry = vx / omega
        cos = np.cos(omega)
        sin = np.sin(omega)
        Hq[0, 0] = cos
        Hq[0, 1] = -sin
        Hq[1, 0] = sin
        Hq[1, 1] = cos
        Hq[0, 2] = rx - rx * cos + ry * sin
        Hq[1, 2] = ry - rx * sin - ry * cos

    return Hq


def Adjoint(H):
    """
    Create an adjoint matrix of a given H-matrix
    """
    AdjointH = np.zeros((3, 3))
    AdjointH[0, 0] = 1
    AdjointH[1:, 1:] = H[:2, :2]
    AdjointH[1, 0] = H[1, 2]
    AdjointH[2, 0] = -H[0, 2]

    return AdjointH


def inverseH(H):
    """
    Create an inverse of a given H-matrix
    """
    Hinverse = np.eye(3)
    Hinverse[:2, :2] = H[:2, :2].transpose()
    Hinverse[:2, 2] = matrix_vector_multiply(-Hinverse[:2, :2], H[:2, 2])
    return Hinverse


def matrix_multiply(A, B):
    """
    Function in order to mulitply matrixs while using ulab
    """
    # Check if dimensions match for matrix multiplication
    rows_A, cols_A = A.shape
    rows_B, cols_B = B.shape
    if cols_A != rows_B:
        raise ValueError("Incompatible dimensions for matrix multiplication")

    result = np.zeros((rows_A, cols_B))

    # Perform matrix multiplication 
    for i in range(rows_A):
        for j in range(cols_B):
            sum = 0
            for k in range(cols_A):
                sum += A[i, k] * B[k, j]
            result[i, j] = sum
    return result


def matrix_vector_multiply(matrix, vector):
    """
    Function in order to mulitply matrix 2x2 with vector 2x1 while using ulab
    """
    # Check if the input matrix is 2x2 and vector is 2x1
    if matrix.size != 4 or vector.size != 2:
        raise ValueError("Matrix must be 2x2 and vector must be 2x1")

    # Perform multiplication
    result = np.zeros(2)
    result[0] = matrix[0, 0] * vector[0] + matrix[0, 1] * vector[1]
    result[1] = matrix[1, 0] * vector[0] + matrix[1, 1] * vector[1]

    return result
