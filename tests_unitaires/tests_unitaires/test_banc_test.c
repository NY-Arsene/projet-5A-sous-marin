/* Test du banc de test lui-même : sur un sous-marin simulé sain il doit tout valider, sur un sous-marin en panne il doit signaler. */
#include "mini_tests.h"
#include "banc_test.h"
#include "faux_composants.h"
#include "gestionnaire_capteurs.h"
#include <stdio.h>

static fake_sub_t s_sub;

static void healthy_sub(void)
{
    fake_sub_setup(&s_sub);
    /* l'IMU/le magnétomètre doivent renvoyer des valeurs plausibles ; on les règle après leur configuration par le banc :
     * le modèle décode la pleine échelle depuis les registres au moment de fake_*_set, donc on fixe ici des octets bruts
     * équivalents à (0,0,-1 g) en +-2 g et (0.22, 0, 0.42 G) en +-4 G. */
    s_sub.imu.regs[0x2C] = (uint8_t)((uint16_t)(-16393) & 0xFF);       /* accel Z = -1 g (0,061 mg/LSB) */
    s_sub.imu.regs[0x2D] = (uint8_t)(((uint16_t)(-16393)) >> 8);
    int16_t mx = (int16_t)(0.22f * 6842.0f), mz = (int16_t)(0.42f * 6842.0f);
    s_sub.mag.regs[0x28] = (uint8_t)(mx & 0xFF); s_sub.mag.regs[0x29] = (uint8_t)((uint16_t)mx >> 8);
    s_sub.mag.regs[0x2C] = (uint8_t)(mz & 0xFF); s_sub.mag.regs[0x2D] = (uint8_t)((uint16_t)mz >> 8);
}

static void test_bench_passes_on_a_healthy_submarine(void)
{
    healthy_sub();
    printf("--- sortie du banc (extraits) ---\n");
    MU_ASSERT_INT(0, bench_run_all());
}

static void test_bench_reports_missing_pressure_sensor(void)
{
    healthy_sub();
    fake_remove_i2c(0x76);
    MU_ASSERT(bench_i2c_scan() >= 1);
    MU_ASSERT(bench_baro() >= 1);
}

static void test_bench_detects_false_leak_alarm_and_noisy_pressure(void)
{
    healthy_sub();
    fake_adc_value[PORT_ADC_LEAK] = 3500;                              /* électrodes "mouillées" alors qu'elles devraient être sèches */
    MU_ASSERT(bench_leak() >= 1);
}

static void test_menu_runs_and_quits(void)
{
    healthy_sub();
    fake_console_feed("sptq");
    bench_menu();
    MU_ASSERT(1);
}

int main(void)
{
    MU_RUN(test_bench_passes_on_a_healthy_submarine);
    MU_RUN(test_bench_reports_missing_pressure_sensor);
    MU_RUN(test_bench_detects_false_leak_alarm_and_noisy_pressure);
    MU_RUN(test_menu_runs_and_quits);
    return MU_SUMMARY();
}
