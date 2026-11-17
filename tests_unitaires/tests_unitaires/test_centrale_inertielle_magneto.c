/* Tests des drivers LSM6DSOX (IMU) et LIS3MDL (magnétomètre) face à des modèles de registres écrits d'après les fiches. */
#include "mini_tests.h"
#include "lsm6dsox.h"
#include "lis3mdl.h"
#include "faux_composants.h"
#include <math.h>
#include <string.h>

/* ================================================================== LSM6DSOX */
static void test_imu_init_programs_the_expected_registers(void)
{
    fake_regmap_t m; lsm6dsox_t s;
    fake_imu_setup(&m, 0x6A);
    MU_ASSERT_STATUS(SENS_OK, lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ));
    MU_ASSERT_INT(0x40, m.regs[0x10]);        /* ODR_XL = 0100 (104 Hz), FS_XL = 00 (+-2 g)     */
    MU_ASSERT_INT(0x44, m.regs[0x11]);        /* ODR_G  = 0100 (104 Hz), FS_G  = 01 (500 dps)   */
    MU_ASSERT_INT(0x44, m.regs[0x12]);        /* BDU = 1 et IF_INC = 1                          */
    MU_ASSERT_NEAR(0.000061, s.acc_g_per_lsb, 1e-9);
    MU_ASSERT_NEAR(0.0175, s.gyr_dps_per_lsb, 1e-9);
}

static void test_imu_init_errors(void)
{
    fake_regmap_t m; lsm6dsox_t s;
    fake_imu_setup(&m, 0x6A);
    m.regs[0x0F] = 0x69;                                                /* mauvais composant (LSM6DS3 : 0x69) */
    MU_ASSERT_STATUS(SENS_ERR_ID, lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ));
    m.regs[0x0F] = 0x6C;
    MU_ASSERT_STATUS(SENS_ERR_PARAM, lsm6dsox_init(&s, 0x42, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, (lsm6dsox_odr_t)0));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, lsm6dsox_init(NULL, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ));
    fake_i2c_set_fail(true);
    MU_ASSERT_STATUS(SENS_ERR_BUS, lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ));
}

static void test_imu_init_times_out_if_software_reset_never_completes(void)
{
    fake_regmap_t m; lsm6dsox_t s;
    fake_imu_setup(&m, 0x6A);
    m.on_write_reg = NULL;                                              /* le bit SW_RESET ne retombe jamais */
    uint32_t t0 = fake_time_get();
    MU_ASSERT_STATUS(SENS_ERR_TIMEOUT, lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ));
    MU_ASSERT(fake_time_get() - t0 <= 50);                              /* l'attente est bornée */
}

static void test_imu_read_scaling_and_signs(void)
{
    fake_regmap_t m; lsm6dsox_t s; lsm6dsox_data_t d;
    fake_imu_setup(&m, 0x6A);
    lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ);
    const float acc[3] = { -0.5f, 0.25f, -1.0f }, gyr[3] = { 100.0f, -50.0f, 0.5f };
    fake_imu_set(&m, acc, gyr);
    MU_ASSERT_STATUS(SENS_OK, lsm6dsox_read(&s, &d));
    for (int i = 0; i < 3; i++) {
        MU_ASSERT_NEAR(acc[i], d.acc_g[i], 0.001);                      /* valeurs négatives : extension de signe */
        MU_ASSERT_NEAR(gyr[i], d.gyr_dps[i], 0.02);
    }
    MU_ASSERT_INT(-16393, d.acc_raw[2]);
}

static void test_imu_every_full_scale_setting_is_decoded_correctly(void)
{
    const lsm6dsox_acc_fs_t afs[4] = { LSM6_XL_2G, LSM6_XL_4G, LSM6_XL_8G, LSM6_XL_16G };
    const lsm6dsox_gyr_fs_t gfs[4] = { LSM6_G_250DPS, LSM6_G_500DPS, LSM6_G_1000DPS, LSM6_G_2000DPS };
    for (int a = 0; a < 4; a++) {
        for (int g = 0; g < 4; g++) {
            fake_reset();
            fake_regmap_t m; lsm6dsox_t s; lsm6dsox_data_t d;
            fake_imu_setup(&m, 0x6B);
            MU_ASSERT_STATUS(SENS_OK, lsm6dsox_init(&s, 0x6B, afs[a], gfs[g], LSM6_ODR_208HZ));
            const float acc[3] = { 0.5f, -0.3f, 1.0f }, gyr[3] = { 10.0f, -20.0f, 5.0f };
            fake_imu_set(&m, acc, gyr);
            MU_ASSERT_STATUS(SENS_OK, lsm6dsox_read(&s, &d));
            for (int i = 0; i < 3; i++) {
                MU_ASSERT_NEAR(acc[i], d.acc_g[i], 3 * s.acc_g_per_lsb);
                MU_ASSERT_NEAR(gyr[i], d.gyr_dps[i], 3 * s.gyr_dps_per_lsb);
            }
        }
    }
}

static void test_imu_not_ready_unless_both_accel_and_gyro_ready(void)
{
    fake_regmap_t m; lsm6dsox_t s; lsm6dsox_data_t d;
    fake_imu_setup(&m, 0x6A);
    lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ);
    m.regs[0x1E] = 0x00; MU_ASSERT_STATUS(SENS_ERR_NOT_READY, lsm6dsox_read(&s, &d));
    m.regs[0x1E] = 0x01; MU_ASSERT_STATUS(SENS_ERR_NOT_READY, lsm6dsox_read(&s, &d));   /* accel seulement */
    m.regs[0x1E] = 0x02; MU_ASSERT_STATUS(SENS_ERR_NOT_READY, lsm6dsox_read(&s, &d));   /* gyro seulement  */
    m.regs[0x1E] = 0x03; MU_ASSERT_STATUS(SENS_OK, lsm6dsox_read(&s, &d));
    fake_i2c_set_fail(true);
    MU_ASSERT_STATUS(SENS_ERR_BUS, lsm6dsox_read(&s, &d));
}

static void test_imu_gyro_calibration_removes_bias(void)
{
    fake_regmap_t m; lsm6dsox_t s; lsm6dsox_data_t d;
    fake_imu_setup(&m, 0x6A);
    lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ);
    const float acc[3] = { 0, 0, -1 }, bias[3] = { 1.5f, -0.8f, 0.3f };
    fake_imu_set(&m, acc, bias);
    MU_ASSERT_STATUS(SENS_OK, lsm6dsox_calibrate_gyro(&s, 200, 2000));
    MU_ASSERT_NEAR(1.5, s.gyro_bias_dps[0], 0.02);
    MU_ASSERT_NEAR(-0.8, s.gyro_bias_dps[1], 0.02);
    lsm6dsox_read(&s, &d);
    MU_ASSERT_NEAR(0.0, d.gyr_dps[0], 0.03);                            /* au repos : sortie nulle après calibration */
    MU_ASSERT_NEAR(0.0, d.gyr_dps[2], 0.03);
    const float rot[3] = { 31.5f, -0.8f, 0.3f };                        /* vraie rotation de 30 deg/s en plus du biais */
    fake_imu_set(&m, acc, rot);
    lsm6dsox_read(&s, &d);
    MU_ASSERT_NEAR(30.0, d.gyr_dps[0], 0.05);
}

static float s_toggle;
static void shake_hook(fake_regmap_t *m, uint8_t reg)                   /* le gyro change à chaque lecture : le module bouge */
{
    if (reg == 0x22) {
        s_toggle = -s_toggle;
        const float acc[3] = { 0, 0, -1 }, gyr[3] = { s_toggle, 0, 0 };
        fake_imu_set(m, acc, gyr);
    }
}

static void test_imu_gyro_calibration_refuses_when_moving(void)
{
    fake_regmap_t m; lsm6dsox_t s;
    fake_imu_setup(&m, 0x6A);
    lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ);
    s_toggle = 5.0f;
    m.on_read_reg = shake_hook;                                         /* +-5 deg/s alternés */
    MU_ASSERT_STATUS(SENS_ERR_RANGE, lsm6dsox_calibrate_gyro(&s, 100, 2000));
    MU_ASSERT_NEAR(0.0, s.gyro_bias_dps[0], 0);                         /* l'ancien biais est conservé */
}

static void test_imu_gyro_calibration_timeout_and_params(void)
{
    fake_regmap_t m; lsm6dsox_t s;
    fake_imu_setup(&m, 0x6A);
    lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ);
    m.regs[0x1E] = 0x00;                                                /* aucune donnée jamais prête */
    uint32_t t0 = fake_time_get();
    MU_ASSERT_STATUS(SENS_ERR_TIMEOUT, lsm6dsox_calibrate_gyro(&s, 100, 500));
    MU_ASSERT(fake_time_get() - t0 < 600);
    MU_ASSERT_STATUS(SENS_ERR_PARAM, lsm6dsox_calibrate_gyro(&s, 1, 500));
}

static void test_imu_gyro_calibration_reports_bus_error(void)
{
    fake_regmap_t m; lsm6dsox_t s;
    fake_imu_setup(&m, 0x6A);
    lsm6dsox_init(&s, 0x6A, LSM6_XL_2G, LSM6_G_500DPS, LSM6_ODR_104HZ);
    fake_i2c_set_fail(true);
    MU_ASSERT_STATUS(SENS_ERR_BUS, lsm6dsox_calibrate_gyro(&s, 100, 500));
}

/* ================================================================== LIS3MDL */
static void test_mag_init_programs_the_expected_registers(void)
{
    fake_regmap_t m; lis3mdl_t s;
    fake_mag_setup(&m, 0x1C);
    MU_ASSERT_STATUS(SENS_OK, lis3mdl_init(&s, 0x1C, LIS3_FS_4G, LIS3_ODR_80HZ));
    MU_ASSERT_INT(0x7C, m.regs[0x20]);        /* XY ultra-perf (11), ODR 80 Hz (111), FAST_ODR 0 */
    MU_ASSERT_INT(0x00, m.regs[0x21]);        /* +-4 gauss                                        */
    MU_ASSERT_INT(0x00, m.regs[0x22]);        /* mode continu                                     */
    MU_ASSERT_INT(0x0C, m.regs[0x23]);        /* Z ultra-perf                                     */
    MU_ASSERT_INT(0x40, m.regs[0x24]);        /* BDU                                              */
    MU_ASSERT_STATUS(SENS_OK, lis3mdl_init(&s, 0x1C, LIS3_FS_8G, LIS3_ODR_10HZ));
    MU_ASSERT_INT(0x20, m.regs[0x21]);
    MU_ASSERT_INT(0x70, m.regs[0x20]);
}

static void test_mag_init_errors(void)
{
    fake_regmap_t m; lis3mdl_t s;
    fake_mag_setup(&m, 0x1C);
    m.regs[0x0F] = 0x00;
    MU_ASSERT_STATUS(SENS_ERR_ID, lis3mdl_init(&s, 0x1C, LIS3_FS_4G, LIS3_ODR_80HZ));
    m.regs[0x0F] = 0x3D;
    MU_ASSERT_STATUS(SENS_ERR_PARAM, lis3mdl_init(&s, 0x1D, LIS3_FS_4G, LIS3_ODR_80HZ));
    MU_ASSERT_STATUS(SENS_ERR_PARAM, lis3mdl_init(&s, 0x1C, (lis3mdl_fs_t)7, LIS3_ODR_80HZ));
    fake_i2c_set_fail(true);
    MU_ASSERT_STATUS(SENS_ERR_BUS, lis3mdl_init(&s, 0x1C, LIS3_FS_4G, LIS3_ODR_80HZ));
}

static void test_mag_read_uses_auto_increment_and_scales_each_range(void)
{
    const lis3mdl_fs_t fs[4] = { LIS3_FS_4G, LIS3_FS_8G, LIS3_FS_12G, LIS3_FS_16G };
    for (int k = 0; k < 4; k++) {
        fake_reset();
        fake_regmap_t m; lis3mdl_t s; lis3mdl_data_t d;
        fake_mag_setup(&m, 0x1E);
        MU_ASSERT_STATUS(SENS_OK, lis3mdl_init(&s, 0x1E, fs[k], LIS3_ODR_40HZ));
        const float g[3] = { 0.25f, -0.40f, 0.10f };                     /* trois valeurs DIFFÉRENTES : détecte un oubli d'auto-incrément */
        fake_mag_set(&m, g);
        MU_ASSERT_STATUS(SENS_OK, lis3mdl_read(&s, &d));
        for (int i = 0; i < 3; i++) MU_ASSERT_NEAR(g[i], d.gauss[i], 0.002);
    }
}

static void test_mag_not_ready_and_bus_error(void)
{
    fake_regmap_t m; lis3mdl_t s; lis3mdl_data_t d;
    fake_mag_setup(&m, 0x1C);
    lis3mdl_init(&s, 0x1C, LIS3_FS_4G, LIS3_ODR_80HZ);
    m.regs[0x27] = 0x00;
    MU_ASSERT_STATUS(SENS_ERR_NOT_READY, lis3mdl_read(&s, &d));
    m.regs[0x27] = 0x07;                                                /* X,Y,Z prêts mais pas ZYXDA */
    MU_ASSERT_STATUS(SENS_ERR_NOT_READY, lis3mdl_read(&s, &d));
    m.regs[0x27] = 0x08;
    MU_ASSERT_STATUS(SENS_OK, lis3mdl_read(&s, &d));
    fake_remove_i2c(0x1C);
    MU_ASSERT_STATUS(SENS_ERR_BUS, lis3mdl_read(&s, &d));
}

int main(void)
{
    MU_RUN(test_imu_init_programs_the_expected_registers);
    MU_RUN(test_imu_init_errors);
    MU_RUN(test_imu_init_times_out_if_software_reset_never_completes);
    MU_RUN(test_imu_read_scaling_and_signs);
    MU_RUN(test_imu_every_full_scale_setting_is_decoded_correctly);
    MU_RUN(test_imu_not_ready_unless_both_accel_and_gyro_ready);
    MU_RUN(test_imu_gyro_calibration_removes_bias);
    MU_RUN(test_imu_gyro_calibration_refuses_when_moving);
    MU_RUN(test_imu_gyro_calibration_timeout_and_params);
    MU_RUN(test_imu_gyro_calibration_reports_bus_error);
    MU_RUN(test_mag_init_programs_the_expected_registers);
    MU_RUN(test_mag_init_errors);
    MU_RUN(test_mag_read_uses_auto_increment_and_scales_each_range);
    MU_RUN(test_mag_not_ready_and_bus_error);
    return MU_SUMMARY();
}
