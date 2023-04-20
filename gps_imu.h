#include "neom8n.h"

void multiplyMatrix(int first[][4], int second[][4], int result[][4])
{
    int i, j, k;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            result[i][j] = 0;
            for (k = 0; k < 4; k++)
            {
                result[i][j] += first[i][k] * second[k][j];
            }
        }
    }
}

void addMatrix(int first[][4], int second[][4], int result[][4])
{
    int i, j;
    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            result[i][j] = first[i][j] + second[i][j];
        }
    }
}

float result[4][4];
float delT = 0.5;
float lat_corr, lon_corr, Vx_corr, Vy_corr, V_corr;
float xichma_accX = 0.00;
float xichma_accY = 0.06;
float xichma_velX = xichma_accX * delT;                                         // = 0.1
float xichma_velY = xichma_accY * delT;                                         // = -0.02
float xichma_posX = (xichma_accX * delT * delT) / 2;                            // = 0.05
float xichma_posY = (xichma_accY * delT * delT) / 2;                            // = -0.01
float xichma_pos = sqrt(xichma_posX * xichma_posX + xichma_posY * xichma_posY); // = 0.051
// float xichma_vel = sqrt(xichma_velX*xichma_velX + xichma_velY*xichma_velY);

float Q_k[4][4] = {
    {xichma_posX * xichma_posX, 0, xichma_posX *xichma_velX, 0},
    {0, xichma_posY *xichma_posY, 0, xichma_posY *xichma_velY},
    {0, 0, xichma_velX *xichma_velX, 0},
    {0, 0, 0, xichma_velY *xichma_velY}};

float H_k[2][4] = {
    {1, 0, 0, 0},
    {0, 1, 0, 0},
};

float H_k_T[4][2] = {
    {1, 0},
    {0, 1},
    {0, 0},
    {0, 0},
};

float R_k[2][2] = {
    {xichma_pos * xichma_pos, 0},
    {0, xichma_pos *xichma_pos}};

void KalmanFilter(float Ax, float Ay, float lat, float lon)
{
    float X_k[4][1] = {lat,
                       lon,
                       0,
                       0};

    float Y_k[2][1] = {lat,
                       lon};

    float F_k[4][4] = {
        {1, 0, delT, 0},
        {0, 1, 0, delT},
        {0, 0, 1, 0},
        {0, 0, 0, 1}};

    float F_k_T[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {delT, 0, 1, 0},
        {0, delT, 0, 1}};

    float B_k[4][2] = {
        {(delT * delT) / 2, 0},
        {0, (delT * delT) / 2},
        {delT, 0},
        {0, delT}};

    float U_k[2][1] = {Ax,
                       Ay};

    float P_k[4][4] = {
        {0.01, 0, 0, 0},
        {0, 0.01, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1},
    };

    //*****************************************************
    //                  PREDICTION
    //*****************************************************
    // X_Pre = F_k*X_k + B_k*U_k
    // X_Pre = C1 + D1

    float C1[4][1];
    float D1[4][1];
    float X_Pre[4][1];
    float C2[4][4];
    float D2[4][4];
    float P_Pre[4][4];
    float C3[4][2];
    float C4[2][4];
    float D3_phuhop[2][2];
    float D4[2][2];
    float K[4][2];
    float C5[2][1];
    float D5[2][1];
    float C6[4][1];
    float C7[4][4];
    float X_Corr[4][1];
    float D7[4][4];
    float D3_dao[2][2];
    // C1 = F_k*X_k
    multiplyMatrix(F_k, X_k, C1);

    // D1 = B_k*U_k
    multiplyMatrix(B_k, U_k, D1);

    // X_Pre=C1+D1
    addMatrix(C1, D1, X_Pre);

    // P_Pre = F_k *P_k *F_k_T + Q_k
    // P_Pre = C2*F_k_T + Q_k
    // P_Pre = D2 + Q_k

    // C2 = F_k*P_k
    multiplyMatrix(F_k, P_k, C2);

    // D2 = C2*F_k_T
    multiplyMatrix(C2, F_k_T, D2);

    // P= D2 + Q_k
    addMatrix(D2, Q_k, P_Pre);
    //*****************************************************
    //                  CORRECTION
    //*****************************************************
    // K = P_Pre *H_k_T *(H_k*P_Pre*H_k_T + R_k)^(-1)
    // K = C3*(H_k*P_Pre*H_k_T + R_k)^(-1)
    // K = C3*D3

    // C3 = P_Pre*H_k_T
    multiplyMatrix(P_Pre, H_k_T, C3);

    // D3 = H_k*P_Pre*H_k_T + R_k
    // D3 = C4*H_k_T + R_k

    // C4 = H_k*P_Pre
    multiplyMatrix(H_k, P_Pre, C4);

    // D4 = C4*H_k_T
    multiplyMatrix(H_k, P_Pre, C4);

    // D3 = D4 + R_k
    multiplyMatrix(H_k, P_Pre, C4);

    // D3(-1), find det(D3), matrix transform, maxtrix phu. hop., D3(-1) = (1/det_D3)*matrix phu. hop.;
    // tinh det_D3
    float det_D3 = D3[0][0] * D3[1][1] - D3[1][0] * D3[0][1];
    // tim ma tran phu hop cua D3: D3_phuhop
    D3_phuhop[0][0] = D3[1][1];
    D3_phuhop[0][1] = -D3[0][1];
    D3_phuhop[1][0] = -D3[1][0];
    D3_phuhop[1][1] = D3[0][0];
    // tim ma tran nghich dao cua D3: D3_dao
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            D3_dao[i][j] = D3_phuhop[i][j] / det_D3;
        }
    }

    // K = C3*D3(-1)
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            K[i][j] = 0;
            for (int k = 0; k < 2; k++)
            {
                K[i][j] += C3[i][k] * D3_dao[k][j];
            }
        }
    }

    // X_Corr = X_Pre + K*(Y_k - H_k*X_Pre)
    // X_Corr = X_Pre + K*(Y_k - C5)

    // C5 = H_k*X_Pre
    multiplyMatrix(H_k, X_Pre, C5);

    // D5 = Y_k - C5
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 1; j++)
        {
            D5[i][j] = Y_k[i][j] - C5[i][j];
        }
    }

    // C6 = K*D5
    multiplyMatrix(K, D5, C6);

    // X_Corr = X_Pre + C6
    addMatrix(X_Pre, C6, X_Corr);
    // P_Corr = (1-K*H_k)*P_Pre
    // P_Corr = (1-C7)*P_Pre

    // C7 = K*H_k
    multiplyMatrix(K, H_k, C7);

    // D7 = 1 - C7
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            D7[i][j] = 1 - C7[i][j];
        }
    }

    // P_Corr = D7*P_Pre
    // float P_k[4][4];
    multiplyMatrix(D7, P_Pre, P_k);

    // In ma tran
    // Get data from matrix
    lat_corr = X_Corr[0][0];
    lon_corr = X_Corr[1][0];
    Vx_corr = X_Corr[2][0] * 9.81;
    Vy_corr = X_Corr[3][0] * 9.81;
    V_corr = sqrt(Vx_corr * Vx_corr + Vy_corr * Vy_corr) * 3.6; // convert m/s -> km/h
}