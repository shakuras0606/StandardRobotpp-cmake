#include "chassis_balance_extras.h"

#if (CHASSIS_TYPE == CHASSIS_BALANCE)
#include "math.h"

static float ClampUnit(float x)
{
    if (x > 1.0f) return 1.0f;
    if (x < -1.0f) return -1.0f;
    return x;
}

/**
 * @brief 获取K矩阵
 * @param[in]  l 腿长
 * @param[out] k K矩阵
 */
void GetK(float l, float k[2][6], bool is_take_off)
{
    float t1 = l;
    float t2 = l * l;
    float t3 = l * l * l;
    // k[0][0] = -157.0203f * t3 + 179.8485f * t2 - 85.4172f * t1 + 0.0537f;
    // k[0][1] = -1.0899f * t3 + 3.0072f * t2 - 6.6735f * t1 + 0.1905f;
    // k[0][2] = -27.7037f * t3 + 27.7682f * t2 - 9.7108f * t1 - 0.2497f;
    // k[0][3] = -32.3706f * t3 + 32.7508f * t2 - 12.0579f * t1 - 0.3533f;
    // k[0][4] = -38.5250f * t3 + 54.5544f * t2 - 30.4327f * t1 + 8.2300f;
    // k[0][5] = -5.2760f * t3 + 8.9537f * t2 - 5.7839f * t1 + 1.8671f;
    // k[1][0] = 152.7053f * t3 - 130.2649f * t2 + 28.7526f * t1 + 5.6772f;
    // k[1][1] = 16.2673f * t3 - 16.3037f * t2 + 5.2455f * t1 + 0.1397f;
    // k[1][2] = -21.5602f * t3 + 27.7447f * t2 - 13.9335f * t1 + 3.1997f;
    // k[1][3] = -24.4104f * t3 + 31.6150f * t2 - 16.0527f * t1 + 3.7713f;
    // k[1][4] = 224.1583f * t3 - 227.6099f * t2 + 81.5551f * t1 + 0.1116f;
    // k[1][5] = 50.1510f * t3 - 51.5745f * t2 + 18.8992f * t1 - 0.3485f;



    // k[0][0] = -109.5870f*t3 + 123.6211f*t2-56.5136f*t1-1.2538f;
    // k[0][1] = -1.0212f*t3 + 2.0317f*t2-3.4901f*t1-0.0701f;
    // k[0][2] = -0.9661f*t3 + 0.8956f*t2-0.2726f*t1-0.1859f;
    // k[0][3] = -3.0585f*t3 + 2.8425f*t2-0.9140f*t1-0.6624f;
    // k[0][4] = -70.9322f*t3 + 79.5699f*t2-34.4547f*t1 + 7.0306f;
    // k[0][5] = -11.5707f*t3 + 12.7168f*t2-5.3307f*t1 + 1.1245f;
    // k[1][0] = 53.3160f*t3-47.7171f*t2 + 13.8458f*t1 + 3.4542f;
    // k[1][1] = -1.6117f*t3 + 1.7616f*t2-1.0313f*t1 + 0.2970f;
    // k[1][2] = -4.1688f*t3 + 4.4382f*t2-1.7288f*t1 + 0.2378f;
    // k[1][3] = -14.7983f*t3 + 15.6955f*t2-6.0890f*t1 + 0.8197f;
    // k[1][4] = 111.2499f*t3-110.2837f*t2 + 38.5958f*t1 + 5.8381f;
    // k[1][5] = 17.7848f*t3-17.4648f*t2 + 5.9975f*t1 + 1.1169f;


//无头
// k[0][0] = -126.7559f*t3 + 147.6532f*t2 - 71.7502f*t1 + 0.6372f;
// k[0][1] = -0.7901f*t3 + 2.2012f*t2 - 5.2267f*t1 + 0.1644f;
// k[0][2] = -30.1471f*t3 + 30.1122f*t2 - 10.4339f*t1 - 0.0920f;
// k[0][3] = -33.8748f*t3 + 34.1001f*t2 - 12.3483f*t1 - 0.1479f;
// k[0][4] = -23.7764f*t3 + 42.9350f*t2 - 28.1571f*t1 + 8.5427f;
// k[0][5] = 1.2743f*t3 + 0.6637f*t2 - 1.5932f*t1 + 0.8708f;
// k[1][0] = 248.1533f*t3 - 210.4801f*t2 + 46.6631f*t1 + 8.0009f;
// k[1][1] = 24.7232f*t3 - 23.9955f*t2 + 7.2061f*t1 + 0.2734f;
// k[1][2] = -35.7916f*t3 + 48.0447f*t2 - 24.6947f*t1 + 5.5995f;
// k[1][3] = -39.8180f*t3 + 53.5417f*t2 - 27.6953f*t1 + 6.4114f;
// k[1][4] = 460.8582f*t3 - 469.8111f*t2 + 168.8882f*t1 - 4.9403f;
// k[1][5] = 45.3422f*t3 - 47.6636f*t2 + 18.0232f*t1 - 1.1667f;
//有头
// k[0][0] = -185.5257f*t3 + 223.7988f*t2 - 109.4750f*t1 + 1.7442f;
// k[0][1] = 1.0980f*t3 + 1.3825f*t2 - 6.1080f*t1 + 0.2415f;
// k[0][2] = -34.2564f*t3 + 35.1136f*t2 - 12.6260f*t1 + 0.2716f;
// k[0][3] = -46.2613f*t3 + 47.6281f*t2 - 17.5521f*t1 + 0.3288f;
// k[0][4] = 22.4234f*t3 + 4.6166f*t2 - 20.7574f*t1 + 9.8505f;
// k[0][5] = 6.4876f*t3 - 3.5937f*t2 - 0.8460f*t1 + 1.0769f;
// k[1][0] = 525.6401f*t3 - 463.8019f*t2 + 115.0496f*t1 + 11.0489f;
// k[1][1] = 39.7301f*t3 - 40.0073f*t2 + 12.8805f*t1 + 0.1506f;
// k[1][2] = -12.4156f*t3 + 27.7869f*t2 - 19.8884f*t1 + 5.6921f;
// k[1][3] = -15.5961f*t3 + 36.3559f*t2 - 26.5699f*t1 + 7.7683f;
// k[1][4] = 586.8379f*t3 - 619.1333f*t2 + 233.8326f*t1 - 12.4723f;
// k[1][5] = 62.7862f*t3 - 68.0756f*t2 + 26.8498f*t1 - 2.0168f;

//0.5mass
k[0][0] = -68.2016f*t3 + 90.5219f*t2 - 57.1125f*t1 + 0.7436f;
k[0][1] = 4.2234f*t3 - 3.2039f*t2 - 4.7369f*t1 + 0.1626f;
k[0][2] = -34.1765f*t3 + 35.9991f*t2 - 13.5141f*t1 + 0.1233f;
k[0][3] = -40.6682f*t3 + 43.4937f*t2 - 17.3559f*t1 + 0.0773f;
k[0][4] = -0.1884f*t3 + 19.9584f*t2 - 21.2772f*t1 + 8.5155f;
k[0][5] = 3.8797f*t3 - 1.0694f*t2 - 1.7687f*t1 + 1.3174f;
k[1][0] = 209.4388f*t3 - 194.4450f*t2 + 53.5711f*t1 + 4.3069f;
k[1][1] = 22.5592f*t3 - 24.1141f*t2 + 9.0272f*t1 + 0.1110f;
k[1][2] = -8.3755f*t3 + 21.4966f*t2 - 16.6374f*t1 + 5.3722f;
k[1][3] = -8.7739f*t3 + 24.3648f*t2 - 19.5939f*t1 + 6.7039f;
k[1][4] = 327.8819f*t3 - 348.7782f*t2 + 133.4940f*t1 - 4.1565f;
k[1][5] = 47.4581f*t3 - 52.0896f*t2 + 20.9699f*t1 - 1.4308f;

// k[0][0] = -149.1512f*t3 + 179.3776f*t2-110.9641f*t1-2.7112f;
// k[0][1] = 0.0916f*t3 + 1.2195f*t2-1.7353f*t1 + 0.0665f;
// k[0][2] = 0.0000f*t3-0.0000f*t2-0.0000f*t1 + 0.4767f;
// k[0][3] = 0.1954f*t3-0.2532f*t2 + 0.2263f*t1 + 1.1260f;
// k[0][4] = -62.8137f*t3 + 83.5624f*t2-70.4549f*t1-20.5016f;
// k[0][5] = -6.2711f*t3 + 8.6409f*t2-7.9214f*t1-2.8005f;
// k[1][0] = 0.0000f*t3-0.0000f*t2 + 0.0000f*t1 + 0.0000f;
// k[1][1] = -0.0000f*t3-0.0000f*t2-0.0000f*t1-0.0000f;
// k[1][2] = -0.0000f*t3 + 0.0000f*t2-0.0000f*t1-0.0000f;
// k[1][3] = 0.0000f*t3 + 0.0000f*t2-0.0000f*t1-0.0000f;
// k[1][4] = -0.0000f*t3 + 0.0000f*t2 + 0.0000f*t1 + 0.0000f;
// k[1][5] = -0.0000f*t3 + 0.0000f*t2 + 0.0000f*t1 + 0.0000f;
    
    // if (is_take_off) {
    //     k[0][0] = 0;
    //     k[0][1] = 0;
    //     k[0][2] = 0;
    //     k[0][3] = 0;
    //     k[0][4] = 0;
    //     k[0][5] = 0;
    //     // k[1][0] = 0;
    //     // k[1][1] = 0;
    //     k[1][2] = 0;
    //     k[1][3] = 0;
    //     k[1][4] = 0;
    //     k[1][5] = 0;
    // }
}

/**
 * @brief 通过关节phi1和phi4的值获取L0和Phi0
 * @param[in]  phi1
 * @param[in]  phi4
 * @param[out] L0_Phi0 L0和Phi0
 */
void GetL0AndPhi0(float phi1, float phi4, float L0_Phi0[2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, phi2, XC, YC;
    float L0, Phi0;
    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), (A0 + C0));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    Phi0 = atan2(YC, (XC - LEG_L5 / 2));

    L0_Phi0[0] = L0;
    L0_Phi0[1] = Phi0;
}

/**
 * @brief 获取dL0和dPhi0
 * @param[in]  J 雅可比矩阵
 * @param[in]  d_phi1 
 * @param[in]  d_phi4 
 */
void GetdL0AnddPhi0(float J[2][2], float d_phi1, float d_phi4, float dL0_dPhi0[2])
{
    // clang-format off
    float d_l0   = J[0][0] * d_phi1 + J[0][1] * d_phi4;
    float d_phi0 = J[1][0] * d_phi1 + J[1][1] * d_phi4;
    // clang-format on
    dL0_dPhi0[0] = d_l0;
    dL0_dPhi0[1] = d_phi0;
}

/**
 * @brief 获取腿部摆杆的等效力
 * @param[in]  J 雅可比矩阵
 * @param[in]  T1 
 * @param[in]  T2 
 * @param[out] F 0-F0 1-Tp
 */
void GetLegForce(float J[2][2], float T1, float T2, float F[2])
{
    float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
    // clang-format off
    float inv_J[4] = {J[1][1] / det, -J[0][1] / det, 
                     -J[1][0] / det,  J[1][1] / det};
    // clang-format on
    //F = (inv_J.') * T
    float F0 = inv_J[0] * T1 + inv_J[2] * T2;
    float Tp = inv_J[1] * T1 + inv_J[3] * T2;

    F[0] = F0;
    F[1] = Tp;
}

/**
 * @brief 计算雅可比矩阵
 * @param phi1 
 * @param phi4 
 * @param J 
 */
void CalcJacobian(float phi1, float phi4, float J[2][2])
{
    float YD, YB, XD, XB, lBD, A0, B0, C0, XC, YC;
    float phi2, phi3;
    float L0, phi0;
    float j11, j12, j21, j22;

    YD = LEG_L4 * sin(phi4);
    YB = LEG_L1 * sin(phi1);
    XD = LEG_L5 + LEG_L4 * cos(phi4);
    XB = LEG_L1 * cos(phi1);
    lBD = sqrt((XD - XB) * (XD - XB) + (YD - YB) * (YD - YB));
    A0 = 2 * LEG_L2 * (XD - XB);
    B0 = 2 * LEG_L2 * (YD - YB);
    C0 = LEG_L2 * LEG_L2 + lBD * lBD - LEG_L3 * LEG_L3;
    phi2 = 2 * atan2((B0 + sqrt(A0 * A0 + B0 * B0 - C0 * C0)), A0 + C0);
    phi3 = atan2(YB - YD + LEG_L2 * sin(phi2), XB - XD + LEG_L2 * cos(phi2));
    XC = LEG_L1 * cos(phi1) + LEG_L2 * cos(phi2);
    YC = LEG_L1 * sin(phi1) + LEG_L2 * sin(phi2);
    L0 = sqrt((XC - LEG_L5 / 2) * (XC - LEG_L5 / 2) + YC * YC);
    phi0 = atan2(YC, XC - LEG_L5 / 2);

    j11 = (LEG_L1 * sin(phi0 - phi3) * sin(phi1 - phi2)) / sin(phi3 - phi2);
    j12 = (LEG_L4 * sin(phi0 - phi2) * sin(phi3 - phi4)) / sin(phi3 - phi2);
    j21 = (LEG_L1 * cos(phi0 - phi3) * sin(phi1 - phi2)) / (L0 * sin(phi3 - phi2));
    j22 = (LEG_L4 * cos(phi0 - phi2) * sin(phi3 - phi4)) / (L0 * sin(phi3 - phi2));

    J[0][0] = j11;
    J[0][1] = j12;
    J[1][0] = j21;
    J[1][1] = j22;
}

/**
 * @brief 计算VMC
 * @param[in]  F0 沿杆方向的力
 * @param[in]  Tp 髋关节力矩
 * @param[in]  J 雅可比矩阵
 * @param[out] T 2个关节的输出力矩
 */
void CalcVmc(float F0, float Tp, float J[2][2], float T[2])
{
    // clang-format off
    float JT[2][2] = {{J[0][0],J[1][0]}, // 转置矩阵
                      {J[0][1],J[1][1]}};
    float F[2] = {F0, Tp};
    // clang-format on
    float T1 = JT[0][0] * F[0] + JT[0][1] * F[1];
    float T2 = JT[1][0] * F[0] + JT[1][1] * F[1];

    T[0] = T1;
    T[1] = T2;
}

/**
 * @brief 通过L0和Phi0的值计算关节phi1和phi4
 * @param[in]  phi0
 * @param[in]  l0
 * @param[out] phi1_phi4 phi1和phi4
 * @note 用于位置控制时求逆解
 */
void CalcPhi1AndPhi4(float phi0, float l0, float phi1_phi4[2])
{
    float L5_2_pow;
    float Lca2, Lce2;
    float cos_phi11, cos_phi12, cos_phi41, cos_phi42;
    float phi11, phi12, phi41, phi42;
    float phi1, phi4;

    L5_2_pow = (LEG_L5 / 2) * (LEG_L5 / 2);  //(LEG_L5 / 2)^2
    Lca2 = l0 * l0 + L5_2_pow + l0 * LEG_L5 * cos(phi0);
    Lce2 = l0 * l0 + L5_2_pow - l0 * LEG_L5 * cos(phi0);

    cos_phi11 = (L5_2_pow + Lca2 - l0 * l0) / (LEG_L5 * sqrt(Lca2));
    cos_phi12 = (LEG_L1 * LEG_L1 + Lca2 - LEG_L2 * LEG_L2) / (2 * LEG_L1 * sqrt(Lca2));
    cos_phi41 = (L5_2_pow + Lce2 - l0 * l0) / (LEG_L5 * sqrt(Lce2));
    cos_phi42 = (LEG_L4 * LEG_L4 + Lce2 - LEG_L3 * LEG_L3) / (2 * LEG_L4 * sqrt(Lce2));

    phi11 = acos(ClampUnit(cos_phi11));
    phi12 = acos(ClampUnit(cos_phi12));

    phi41 = acos(ClampUnit(cos_phi41));
    phi42 = acos(ClampUnit(cos_phi42));

    phi1 = phi11 + phi12;
    phi4 = M_PI - (phi41 + phi42);

    phi1_phi4[0] = phi1;
    phi1_phi4[1] = phi4;
}

/**
 * @brief 通过当前底盘姿态和目标roll角计算两腿长度期望差值
 * @param[in]  Ld0 (m)当前左右腿长度差值(L0l - L0r)
 * @param[in]  theta0 (rad)当前底盘roll角
 * @param[in]  theta1 (rad)目标roll角
 * @return 两腿长度期望差值(m)(L1l - L1r)
 */
inline float CalcLegLengthDiff(float Ld0, float theta0, float theta1)
{
    return WHEEL_BASE * tanf(theta1) -
           cosf(theta0) / cosf(theta1) * (WHEEL_BASE * tanf(theta0) - Ld0);
}

/**
 * @brief 双腿腿长协调控制，维持腿长目标在范围内，同时尽可能达到两腿目标差值
 * @param[in]  Ll_ref   (m)左腿长度指针
 * @param[in]  Lr_ref   (m)右腿长度指针
 * @param[in]  diff (m)腿长差值
 * @param[in]  add  (m)腿长差值补偿量
 */
void CoordinateLegLength(float * Ll_ref, float * Lr_ref, float diff, float add)
{
    *Ll_ref = *Ll_ref + diff * 0.5f + add;
    *Lr_ref = *Lr_ref - diff * 0.5f - add;

    // float delta = *Ll_ref - *Lr_ref;
    // if (delta > MAX_LEG_LENGTH - MIN_LEG_LENGTH) {
    //     *Ll_ref = MAX_LEG_LENGTH;
    //     *Lr_ref = MIN_LEG_LENGTH;
    //     return;
    // } else if (delta < MIN_LEG_LENGTH - MAX_LEG_LENGTH) {
    //     *Ll_ref = MIN_LEG_LENGTH;
    //     *Lr_ref = MAX_LEG_LENGTH;
    //     return;
    // }

    //先判断短腿范围，再判断长腿范围
    float * short_leg = *Ll_ref < *Lr_ref ? Ll_ref : Lr_ref;
    float * long_leg = *Ll_ref < *Lr_ref ? Lr_ref : Ll_ref;
    float move = 0;
    move = MIN_LEG_LENGTH - *short_leg;
    if (move > 0) {
        *short_leg += move;
        *long_leg += move;
    }
    if (*long_leg > MAX_LEG_LENGTH) {
        *long_leg = MAX_LEG_LENGTH;
    }
}

#endif /* CHASSIS_BALANCE */
