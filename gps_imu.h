#include "neom8n.h"

float delT = 0.5;
float lat_corr,lon_corr,Vx_corr,Vy_corr,V_corr;
float xichma_accX = 0.00;
float xichma_accY = 0.06;
float xichma_velX = xichma_accX*delT; // = 0.1
float xichma_velY = xichma_accY*delT; // = -0.02
float xichma_posX = (xichma_accX*delT*delT)/2; // = 0.05
float xichma_posY = (xichma_accY*delT*delT)/2; // = -0.01
float xichma_pos = sqrt(xichma_posX*xichma_posX + xichma_posY*xichma_posY); // = 0.051 
//float xichma_vel = sqrt(xichma_velX*xichma_velX + xichma_velY*xichma_velY);

float Q_k[4][4] = {
        {xichma_posX*xichma_posX, 0, xichma_posX*xichma_velX, 0},
        {0, xichma_posY*xichma_posY, 0, xichma_posY*xichma_velY},
		{0, 0, xichma_velX*xichma_velX, 0},
		{0, 0, 0, xichma_velY*xichma_velY}
}; 

// float P_k[4][4] = {
//         {0.01, 0, 0, 0},
//         {0, 0.01, 0, 0},
// 		{0, 0, 10, 0},
// 		{0, 0, 0, 10},
// }; 

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

// float R_k[4][4] = {
		// {xichma_pos*xichma_pos, 0, 0, 0},
        // {0, xichma_pos*xichma_pos, 0, 0},
		// {0, 0, xichma_vel*xichma_vel, 0},
		// {0, 0, 0, xichma_vel*xichma_vel}
// };

float R_k[2][2] = {
		{xichma_pos*xichma_pos, 0},
        {0, xichma_pos*xichma_pos}
};

void KalmanFilter(float Ax, float Ay, float lat, float lon)
{
    float X_k[4][1] = {
            lat,
            lon,
            0,
            0
    };
    
    float Y_k[2][1] = {
            lat,
            lon 
    };
    
    float F_k[4][4] = {
            {1, 0, delT, 0   },
            {0, 1, 0,    delT},
            {0, 0, 1,    0},
            {0, 0, 0,    1}
    };

    float F_k_T[4][4] = {
            {1,    0,    0,  0},
            {0,    1,    0,  0},
            {delT, 0,    1,  0},
            {0,    delT, 0,  1}
    };
 
    float B_k[4][2] = {
            {(delT*delT)/2, 0            },
            {0,             (delT*delT)/2},
            {delT,          0            },
            {0,             delT         }
    };
    
    float U_k[2][1] = {
            Ax,
            Ay
    };

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
    // C1 = F_k*X_k
    float C1[4][1];
    for(int i=0; i<4; i++){
        for(int j=0; j<1; j++){
            C1[i][j] = 0;
            for(int k=0; k<4; k++){
                C1[i][j] += F_k[i][k]*X_k[k][j];
            }
        }
    }
    // D1 = B_k*U_k
    float D1[4][1];
    for(int i=0; i<4; i++){
        for(int j=0; j<1; j++){
            D1[i][j] = 0;
            for(int k=0; k<2; k++){
                D1[i][j] += B_k[i][k]*U_k[k][j];
            }
        }
    }
    // X_Pre=C1+D1
    float X_Pre[4][1];
    for (int i = 0; i < 4; i++){
      for (int j = 0; j < 1; j++) {
         X_Pre[i][j] = C1[i][j] + D1[i][j];
      } 
    }
    // P_Pre = F_k * P_k * F_k_T + Q_k
    // P_Pre = C2*F_k_T + Q_k
    // P_Pre = D2 + Q_k
    // C2 = F_k*P_k
    float C2[4][4];
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            C2[i][j] = 0;
            for(int k=0; k<4; k++){
                C2[i][j] += F_k[i][k]*P_k[k][j];
            }
        }
    }
    // D2 = C2*F_k_T
    float D2[4][4];
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            D2[i][j] = 0;
            for(int k=0; k<4; k++){
                D2[i][j] += C2[i][k]*F_k_T[k][j];
            }
        }
    }
    // P= D2 + Q_k 
    float P_Pre[4][4];
    for (int i = 0; i < 4; i++){
      for (int j = 0; j < 4; j++) {
        P_Pre[i][j] = D2[i][j] + Q_k[i][j];
      } 
    }
    // In ma tran
    // printf("Prediction\n");
    // for (int i = 0; i < 4; i++) {
    //   for (int j = 0; j < 1; j++) {
    //      printf(" %0.04f ", X_Pre[i][j]);
    //   }
    //   printf("\n");
    // }
    // printf("\n");
    // for (int i = 0; i < 4; i++) {
    //   for (int j = 0; j < 4; j++) {
    //      printf(" %0.04f ", P_Pre[i][j]);
    //   }
    //   printf("\n");
    // }
    // printf("\n");
    //*****************************************************
    //                  CORRECTION
    //*****************************************************
    // K = P_Pre * H_k_T * (H_k*P_Pre*H_k_T + R_k)^(-1)
    // K = C3*(H_k*P_Pre*H_k_T + R_k)^(-1)
    // K = C3*D3
    // C3 = P_Pre*H_k_T
    float C3[4][2];
    for(int i=0; i<4; i++){
        for(int j=0; j<2; j++){
            C3[i][j] = 0;
            for(int k=0; k<4; k++){
                C3[i][j] += P_Pre[i][k]*H_k_T[k][j];
            }
        }
    }
    // D3 = H_k*P_Pre*H_k_T + R_k
    // D3 = C4*H_k_T + R_k
    // C4 = H_k*P_Pre
    float C4[2][4];
    for(int i=0; i<2; i++){
        for(int j=0; j<4; j++){
            C4[i][j] = 0;
            for(int k=0; k<4; k++){
                C4[i][j] += H_k[i][k]*P_Pre[k][j];
            }
        }
    }
    // D4 = C4*H_k_T
    float D4[2][2];
    for(int i=0; i<2; i++){
        for(int j=0; j<2; j++){
            D4[i][j] = 0;
            for(int k=0; k<4; k++){
                D4[i][j] += C4[i][k]*H_k_T[k][j];
            }
        }
    }
    // D3 = D4 + R_k
    float D3[2][2];
    for (int i = 0; i < 2; i++){
      for (int j = 0; j < 2; j++){
        D3[i][j] = D4[i][j] + R_k[i][j];
      } 
    }
    // D3(-1), find det(D3), matrix transform, maxtrix phu. hop., D3(-1) = (1/det_D3)*matrix phu. hop.; 
    // tinh det_D3
    float det_D3 = D3[0][0]*D3[1][1] - D3[1][0]*D3[0][1];
    //printf("det_D3 = %0.6f\n", det_D3);
    // tim ma tran chuyen vi D3_T
    // float D3_T[2][2];
    // for(int i=0; i<2; i++){
        // for(int j=0; j<2; j++){
            // D3_T[i][j] = D3[j][i];
        // }
    // }
    // tim ma tran phu hop cua D3: D3_phuhop
    float D3_phuhop[2][2];
    D3_phuhop[0][0] = D3[1][1];
    D3_phuhop[0][1] = -D3[0][1];
    D3_phuhop[1][0] = -D3[1][0];
    D3_phuhop[1][1] = D3[0][0];
    // tim ma tran nghich dao cua D3: D3_dao
    float D3_dao[2][2];
    for(int i=0; i<2; i++){
        for(int j=0; j<2; j++){
            D3_dao[i][j] = D3_phuhop[i][j]/det_D3;
        }
    }
    // K = C3*D3(-1)
    float K[4][2];
    for(int i=0; i<4; i++){
        for(int j=0; j<2; j++){
            K[i][j] = 0;
            for(int k=0; k<2; k++){
                K[i][j] += C3[i][k]*D3_dao[k][j];
            }
        }
    }
    
    // X_Corr = X_Pre + K*(Y_k - H_k*X_Pre)
    // X_Corr = X_Pre + K*(Y_k - C5)
    // C5 = H_k*X_Pre 
    float C5[2][1];
    for(int i=0; i<2; i++){
        for(int j=0; j<1; j++){
            C5[i][j] = 0;
            for(int k=0; k<4; k++){
                C5[i][j] += H_k[i][k]*X_Pre[k][j];
            }
        }
    }
    // D5 = Y_k - C5
    float D5[2][1];
    for (int i = 0; i < 2; i++){
      for (int j = 0; j < 1; j++) {
        D5[i][j] = Y_k[i][j] - C5[i][j];
      } 
    }
    // C6 = K*D5
    float C6[4][1];
    for(int i=0; i<4; i++){
        for(int j=0; j<1; j++){
            C6[i][j] = 0;
            for(int k=0; k<2; k++){
                C6[i][j] += K[i][k]*D5[k][j];
            }
        }
    }
    // X_Corr = X_Pre + C6
    float X_Corr[4][1];
    for (int i = 0; i < 4; i++){
      for (int j = 0; j < 1; j++) {
        X_Corr[i][j] = X_Pre[i][j] + C6[i][j];
      } 
    }
    // P_Corr = (1-K*H_k)*P_Pre
    // P_Corr = (1-C7)*P_Pre
    // C7 = K*H_k
    float C7[4][4];
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            C7[i][j] = 0;
            for(int k=0; k<2; k++){
                C7[i][j] += K[i][k]*H_k[k][j];
            }
        }
    }
    // D7 = 1 - C7
    float D7[4][4];
    for (int i = 0; i < 4; i++){
      for (int j = 0; j < 4; j++){
        D7[i][j] = 1 - C7[i][j];
      } 
    }
    //P_Corr = D7*P_Pre
    //float P_k[4][4];
    for(int i=0; i<4; i++){
        for(int j=0; j<4; j++){
            P_k[i][j] = 0;
            for(int k=0; k<4; k++){
                P_k[i][j] += D7[i][k]*P_Pre[k][j];
            }
        }
    }
    // In ma tran
    // printf("Correction\n");
    // for (int i = 0; i < 4; i++) {
    //   for (int j = 0; j < 4; j++) {
    //      printf(" %0.02f ", P_k[i][j]);
    //   }
    //   printf("\n");
    // }
    // Get data from matrix
    lat_corr = X_Corr[0][0];
    lon_corr = X_Corr[1][0];
    Vx_corr = X_Corr[2][0]*9.81;
    Vy_corr = X_Corr[3][0]*9.81;
    V_corr = sqrt(Vx_corr*Vx_corr + Vy_corr*Vy_corr)*3.6; //convert m/s -> km/h
    // printf("lat=%0.5f\n", lat_corr);
    // printf("lon=%0.5f\n", lon_corr);
    // printf("Vx=%0.2f\n", Vx_corr);
    // printf("Vy=%0.2f\n", Vy_corr);
    // printf("V=%0.2f\n", V_corr);
}
